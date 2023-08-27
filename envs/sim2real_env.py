import time
from typing import Optional

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from gymnasium.utils.ezpickle import EzPickle

from camera_librealsense import Camera
from envs.ik_controller import IKController
from franka_robot import FrankaRobot


class Sim2RealEnv(gym.Env, EzPickle):

    def __init__(
            self,
            obj_goal_dist_threshold=0.03,
            obj_gripper_dist_threshold=0.02,
            obj_lost_reward=-0.2,
            collision_reward=-1.,
            scenario=None,
            **kwargs
    ):

        self.obj_goal_dist_threshold = obj_goal_dist_threshold
        self.obj_gripper_dist_threshold = obj_gripper_dist_threshold
        self.obj_lost_reward = obj_lost_reward
        self.collision_reward = collision_reward
        self.scenario = scenario

        self.obstacles = []
        self.reward_sum = 0
        self.col_sum = 0
        self.last_t = time.perf_counter()

        self.avoid_col_init = [0., 0.3, 0., -1.2, 0., 1.5, 0., 0.04]
        self.initial_qpos = [0.0254, -0.2188, -0.0265, -2.6851, -0.0092, 2.4664, 0.0068, 0.04]
        self.obstacle_info = []

        # just for init of IKC
        for i in range(3):
            self.obstacle_info.append({
                'pos': [0, 0, 0],
                'size': [0, 0, 0]
            })

        # initiate camera
        self.cam = Camera()
        self.cam.start()

        # initiate robot
        self.robot = FrankaRobot("192.168.178.12")

        self.n_actions = 4
        # IK controller
        self.IKC = IKController(self.robot.robot_base, self.initial_qpos[:7], self.obstacle_info)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.n_actions,), dtype=np.float32)

        # for velocities
        self.last_grip_pos = self.IKC.forward_kinematics(np.array(self.initial_qpos[:7]))[-1][:3, 3]
        body_pos = self.cam.get_body_pos_cv()
        self.last_object_pos = body_pos['object']
        self.last_obstacle_pos = body_pos['obst']
        # We add some space in z direction so that the robot doesn't hit the flag
        self.goal = body_pos['goal'] + np.array([0., 0., 0.05])

        self.last_t = time.perf_counter()
        self.dt = 0.001

        obs = self._get_obs()

        self.observation_space = spaces.Dict(
            dict(
                desired_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype=np.float64
                ),
                achieved_goal=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype=np.float64
                ),
                observation=spaces.Box(
                    -np.inf, np.inf, shape=obs["observation"].shape, dtype=np.float64
                ),
                object_gripper_dist=spaces.Box(
                    -np.inf, np.inf, shape=obs["object_gripper_dist"].shape, dtype=np.float64
                ),
                collision=spaces.Discrete(2),
                grip_pos=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype=np.float64
                ),
                object_pos=spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype=np.float64
                ),
            )
        )

        EzPickle.__init__(self, **kwargs)

    def step(self, action):

        action = np.clip(action, -1, 1)

        # compute target position
        qpos = self.robot.get_current_q()
        grip_pos = self.IKC.forward_kinematics(qpos[:7])[-1][:3, 3]
        target_pos = (grip_pos + np.clip(action[:3], -1, 1) * 0.02).copy()

        # to avoid hitting the table, we clip the target in z-direciton
        table_edge = 0.44
        if target_pos[2] < table_edge:
            target_pos[2] = table_edge

        # calculate forward kinematics and capsule positions for visualization
        q_res, robot_capsules, obst_capsules = self.IKC.solve(qpos[:7], self.obstacle_info, target_pos)

        calc_pos = self.IKC.forward_kinematics(q_res[:7])[-1][:3, 3]

        action = np.append(q_res, action[3])

        # self._set_action() move robot in play script to check infos first

        obs = self._get_obs()

        # obs['target_pos'] = target_pos

        reward = self.compute_reward(obs["achieved_goal"], self.goal, obs['object_gripper_dist'], obs['collision'])
        self.reward_sum += reward

        terminated = False
        truncated = False
        info = {
            "Success": self._is_success(obs["achieved_goal"], self.goal),
            "ExReward": self.reward_sum,
            "Collisions": self.col_sum,
            "grip_pos": obs['grip_pos'],
            "target_pos": target_pos,
            "object_pos": obs["object_pos"],
            "calc_pos": calc_pos,
            "action": action
        }


        return obs, reward, terminated, truncated, info

    def close(self):
        pass

    def _set_action(self, action):
        self.robot.move_q(action)


    # public getter
    def get_obs(self):
        return self._get_obs()

    def _get_obs(self):
        self.dt = time.perf_counter() - self.last_t
        self.last_t = time.perf_counter()

        # robot TODO: frankx (7 joint angles and 2 gripper positions (should always be the same value)
        robot_qpos, robot_qvel = self.robot.get_current_q_dq()

        # gripper TODO: frankx, The RL algorithm was trained with a +0.1 displacement on the z axis, so we might need to
        # TODO: add it to the EEF pos returned by frankx. Alternatively, we can also use the joint angles and simply calculate the forward kinematics.
        grip_pos = self.IKC.forward_kinematics(robot_qpos[:7])[-1][:3, 3]
        grip_velp = (grip_pos - self.last_grip_pos) / self.dt
        self.last_grip_pos = grip_pos

        body_pos = self.cam.get_body_pos_cv()

        # object TODO: cv2? Positions always from the center and size are half-lengths
        object_pos = body_pos['object']
        object_velp = (object_pos - self.last_object_pos) / self.dt
        self.last_object_pos = object_pos

        # object-gripper
        object_rel_pos = object_pos - grip_pos
        object_gripper_dist = np.linalg.norm(object_rel_pos.ravel())

        obstacles = []
        # Obstacle 1 | Only this one is tracked for now
        pos = body_pos['obst']
        vel = (pos - self.last_obstacle_pos) / self.dt
        self.last_obstacle_pos = pos
        size = [0.02, 0.042, 0.035]
        obstacles.append(np.concatenate([pos, vel, size]))
        # Obstacle 2
        pos = [1.3, 0.75, 0.41]
        vel = [0., 0., 0.]
        size = [0.2, 0.02, 0.005]
        obstacles.append(np.concatenate([pos, vel, size]))
        # Obstacle 3
        pos = [1.1, 0.6, 0.41]
        vel = [0., 0., 0.]
        size = [0.02, 0.02, 0.02]
        obstacles.append(np.concatenate([pos, vel, size]))

        self.obstacle_info = []
        for obst in obstacles:
            self.obstacle_info.append({
                'pos': obst[:3],
                'size': obst[6:]
            })

        obst_states = np.concatenate(obstacles)

        # goal
        achieved_goal = np.squeeze(object_pos.copy())

        obs = np.concatenate(
            [
                robot_qpos,
                robot_qvel,
                grip_pos,
                grip_velp,
                object_pos,
                object_velp,
                obst_states
            ]
        )

        return {
            "observation": obs.copy(),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": self.goal.copy(),
            "object_gripper_dist": object_gripper_dist.copy(),
            "collision": self.col_sum,
            "grip_pos": grip_pos,
            "object_pos": object_pos,
        }

    def compute_reward(self, achieved_goal, desired_goal, object_gripper_dist, collision):
        # Compute distance between goal and the achieved goal.
        rew = self._is_success(achieved_goal, desired_goal) - 1
        # object lost reward
        if object_gripper_dist > self.obj_gripper_dist_threshold:
            rew += self.obj_lost_reward
        # collisions reward
        if collision:
            rew += self.collision_reward
        return rew

    def _is_success(self, achieved_goal, desired_goal):
        d = np.linalg.norm(achieved_goal - desired_goal, axis=-1)
        return (d < self.obj_goal_dist_threshold).astype(np.float32)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None,):

        super().reset(seed=seed)

        # move robot to init pos, first we move upwards to ensure the motion from FKI doesn't collide with anything
        # on its way
        self.robot.move_q(self.avoid_col_init)
        self.robot.move_q(self.initial_qpos)

        self.reward_sum = 0
        self.col_sum = 0

        obs = self._get_obs()

        return obs, {}


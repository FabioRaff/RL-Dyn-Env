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

        self.goal = np.array([0., 0., 0.]) # TODO: set real goal
        self.obstacles = []
        self.reward_sum = 0
        self.col_sum = 0

        # initiate robot    TODO: check
        self.robot = FrankaRobot("192.168.5.12")

        # move robot to init pos
        self.initial_qpos = [0.0254, -0.2188, -0.0265, -2.6851, -0.0092, 2.4664, 0.0068, 0., 0.]
        self.robot.move_q(self.initial_qpos)   # TODO: check

        self.n_actions = 4

        obs = self._get_obs()

        self.action_space = spaces.Box(-1.0, 1.0, shape=(self.n_actions,), dtype=np.float32)
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
            )
        )

        # IK controller TODO: check
        self.IKC = IKController(self.robot.robot_base, self.initial_qpos, self.get_obstacle_info())

        # Camera
        self.cam = Camera()
        self.cam.start()
        self.pre_dists = np.array([None, None])  # TODO: relative dists?

        # for velocities TODO: check
        self.last_grip_pos = self.IKC.forward_kinematics(np.array(self.initial_qpos[:7]))[-1][:3, 3]
        self.last_object_pos = np.zeros(3)
        self.last_t = time.perf_counter()
        self.dt = 0.

        EzPickle.__init__(self, **kwargs)

    def step(self, action):

        action = np.clip(action, -1, 1)

        # get obstacle information
        obstacles = self.get_obstacle_info() # TODO from cv2
        # compute target position
        grip_pos = self.robot.get_current_pose()[:3]  # TODO expect the pose to be in 6D
        target_pos = (grip_pos + np.clip(action[:3], -1, 1) * 0.05).copy()

        # to avoid hitting the table, we clip the target in z-direciton TODO: measure table height + 0.025
        table_edge = 0.425
        if target_pos[2] < table_edge:
            target_pos[2] = table_edge

        qpos = self.robot.get_current_q()[:7] # TODO: check

        # calculate forward kinematics and capsule positions for visualization
        q_res, robot_capsules, obst_capsules = self.IKC.solve(qpos, obstacles, target_pos)

        self._set_action(np.append(q_res, action[3]))

        self._move_obstacles() # TODO: probably won't need this

        obs = self._get_obs() # TODO: with frankx and cv2

        reward = self.compute_reward(obs["achieved_goal"], self.goal, obs['object_gripper_dist'], obs['collision'])
        self.reward_sum += reward

        terminated = False
        truncated = False

        info = {
            "Success": self._is_success(obs["achieved_goal"], self.goal),
            "ExReward": self.reward_sum,
            "Collisions": self.col_sum
        }

        return obs, reward, terminated, truncated, info

    def close(self):
        pass

    def _set_action(self, action):
        # ensure that we don't change the action outside of this scope
        action = action.copy()

        self.robot.move_q(action)   # TODO: this should be an 8D array, might need to map inside robot class

    def _move_obstacles(self):
        # TODO: if needed, we can move the obstacles here
        pass

    def _check_collisions(self):
        # TODO: This would be hard to implement without mujoco, so we probably just have to track it by hand
        return 0

    # public getter
    def get_obs(self):
        return self._get_obs()

    def _get_obs(self):
        # TODO: we might need to calculate velocities
        self.dt = time.perf_counter() - self.last_t
        self.last_t = time.perf_counter()

        # robot TODO: frankx (7 joint angles and 2 gripper positions (should always be the same value)
        robot_qpos, robot_qvel = self.robot.get_current_q_dq()

        # gripper TODO: frankx, The RL algorithm was trained with a +0.1 displacement on the z axis, so we might need to
        # TODO: add it to the EEF pos returned by frankx. Alternatively, we can also use the joint angles and simply calculate the forward kinematics.
        grip_pos = self.IKC.forward_kinematics(robot_qpos[:7])[-1][:3, 3]
        grip_velp = (grip_pos - self.last_grip_pos) / self.dt

        # object TODO: cv2? Positions always from the center and size are half-lengths
        object_pos = np.array([0., 0., 0.])
        object_velp = (object_pos - self.last_object_pos) / self.dt

        # object-gripper
        object_rel_pos = object_pos - grip_pos
        object_gripper_dist = np.linalg.norm(object_rel_pos.ravel())

        # obstacles TODO: cv2. Positions always from the center and size are half-lengths
        obstacles = self.get_obstacle_obs()
        obst_states = np.concatenate(obstacles)

        # goal
        achieved_goal = np.squeeze(object_pos.copy())

        # collisions TODO: we probably have to track this by hand
        self.col_sum += self._check_collisions()

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

        # move robot to init pos
        self.initial_qpos = [0.0254, -0.2188, -0.0265, -2.6851, -0.0092, 2.4664, 0.0068, 0., 0.]
        self.robot.move_q(self.initial_qpos)   # TODO: check

        # TODO: reset relative distances?
        self.pre_dists = np.array([None, None])

        self.reward_sum = 0
        self.col_sum = 0

        obs = self._get_obs()

        return obs, {}

    def get_obstacle_obs(self):
        # TODO: with cv2
        frame = self.cam.get_frame()
        dists, _ = self.cam.get_distance(frame, add_to_frame=False)
        dists -= np.array([0.044, 0.042])  # TODO: real env offsets?
        pos_dif = 0.1  # TODO: real env difs?
        if self.pre_dists.any():
            signs = np.sign(dists - self.pre_dists)
        self.pre_dists = dists
        dyn_obstacles = np.array([[dists[0] - pos_dif + 0.5 + 0.8, 0.1 + 0.75, 0.4, 0.015, 0.017, 0.015],
                                  [dists[1] - pos_dif + 0.5 + 0.8, -0.1 + 0.75, 0.4, 0.015, 0.017, 0.015]])

        # TODO: look at this on site and retrieve the required parameters. We can probably approximate the velocities via time measurements.
        obstacle_obs = []
        # Obstacle 1
        pos = [0., 0., 0.]
        vel = [0., 0., 0.]
        size = [0., 0., 0.]
        obstacle_obs.append(np.concatenate([pos, vel, size]))
        # Obstacle 2
        pos = [0., 0., 0.]
        vel = [0., 0., 0.]
        size = [0., 0., 0.]
        obstacle_obs.append(np.concatenate([pos, vel, size]))
        # Obstacle 3
        pos = [0., 0., 0.]
        vel = [0., 0., 0.]
        size = [0., 0., 0.]
        obstacle_obs.append(np.concatenate([pos, vel, size]))
        return obstacle_obs

    def get_obstacle_info(self):
        # TODO We only need this for the IKC.
        obstacle_obs = self.get_obstacle_obs()
        obstacles = []
        for obst in obstacle_obs:
            obstacles.append({
                'pos': obst[:3],
                'size': obst[6:]
            })
        return obstacles


import gymnasium as gym
from envs.config import register_custom_envs
from pynput import keyboard
import time
import numpy as np

global ctrl
global rot_ctrl


def test_env():
    register_custom_envs()
    control_mode = 'position'

    env = gym.make('RandDynObstEnv-v1', num_obst=3, control_mode='ik_controller', n_substeps=100)
    env.reset()

    global ctrl
    global rot_ctrl
    ctrl = 1
    rot_ctrl = -1

    def on_press(key):
        global ctrl
        global rot_ctrl
        grip_ctrl = .8
        action = [0., 0., 0., 0.]
        try:
            if control_mode == 'position':
                # pos
                if key.char == 'l':
                    action = [0., ctrl, 0., -grip_ctrl]
                elif key.char == 'j':
                    action = [0., -ctrl, 0., -grip_ctrl]
                elif key.char == 'o':
                    action = [ctrl, 0., 0., -grip_ctrl]
                elif key.char == 'u':
                    action = [-ctrl, 0., 0., -grip_ctrl]
                elif key.char == 'i':
                    action = [0., 0., ctrl, -grip_ctrl]
                elif key.char == 'k':
                    action = [0., 0., -ctrl, -grip_ctrl]
                # gripper
                elif key.char == 'n':
                    action = [0., 0., 0., grip_ctrl]
                elif key.char == 'm':
                    action = [0., 0., 0., -grip_ctrl]
            elif control_mode == 'position_rotation':
                # invert direction
                if key.char == 'z':
                    rot_ctrl = -rot_ctrl
                    action = [0., 0., 0., 0., 0., 0., 0., 0.]
                # pos
                if key.char == 'l':
                    action = [0., ctrl, 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'j':
                    action = [0., -ctrl, 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'o':
                    action = [ctrl, 0., 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'u':
                    action = [-ctrl, 0., 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'i':
                    action = [0., 0., ctrl, 0., 0., 0., -grip_ctrl]
                elif key.char == 'k':
                    action = [0., 0., -ctrl, 0., 0., 0., -grip_ctrl]
                # rot
                if key.char == 'y':
                    action = [0., 0., 0., rot_ctrl, 0., 0., -grip_ctrl]
                elif key.char == 'g':
                    action = [0., 0., 0., 0., rot_ctrl, 0., -grip_ctrl]
                elif key.char == 'b':
                    action = [0., 0., 0., 0., 0., rot_ctrl, -grip_ctrl]
                # gripper
                elif key.char == 'n':
                    action = [0., 0., 0., 0., 0., 0., grip_ctrl]
                elif key.char == 'm':
                    action = [0., 0., 0., 0., 0., 0., -grip_ctrl]
            elif control_mode == 'torque':
                # invert direction
                if key.char == 'z':
                    ctrl = -ctrl
                    action = [0., 0., 0., 0., 0., 0., 0., 0.]
                # joints 1-7
                if key.char == 'y':
                    action = [ctrl, 0., 0., 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'u':
                    action = [0., ctrl, 0., 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'i':
                    action = [0., 0., ctrl, 0., 0., 0., 0., -grip_ctrl]
                elif key.char == 'o':
                    action = [0., 0., 0., ctrl, 0., 0., 0., -grip_ctrl]
                elif key.char == 'j':
                    action = [0., 0., 0., 0., ctrl, 0., 0., -grip_ctrl]
                elif key.char == 'k':
                    action = [0., 0., 0., 0., 0., ctrl, 0., -grip_ctrl]
                elif key.char == 'l':
                    action = [0., 0., 0., 0., 0., 0., ctrl, -grip_ctrl]
                # gripper
                elif key.char == 'n':
                    action = [0., 0., 0., 0., 0., 0., 0., grip_ctrl]
                elif key.char == 'm':
                    action = [0., 0., 0., 0., 0., 0., 0., -grip_ctrl]

        except AttributeError as e:
            pass
        env.step(action)
        env.render()
    #
    # for j in range(1):
    #     env.reset()
    #     for i in range(20):
    #         env.step(np.random.rand(4)*2-1)
    #         env.render()


    # print( ' ------------------------------- TIME ------------------------------')
    # for s in sorted(env.unwrapped.IKC.hyper_dict.items(), key=lambda x: x[1]['time']):
    #     print(s)
    # print( ' ------------------------------- ERROR -----------------------------')
    # for s in sorted(env.unwrapped.IKC.hyper_dict.items(), key=lambda x: x[1]['error']):
    #     print(s)
    # print(f'Total Iterations: {env.unwrapped.IKC.nit}')
    # print(f'Total Function Evaluations: {env.unwrapped.IKC.nfev}')

    # Collect events until released
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()



if __name__ == '__main__':
    test_env()

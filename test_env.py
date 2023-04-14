import gymnasium as gym
from envs.config import register_custom_envs
from pynput import keyboard
import time
import numpy as np


def test_env():
    register_custom_envs()

    env = gym.make('RandDynObstEnv-v1', num_obst=3)
    env.reset()

    def on_press(key):
        ctrl = 0.2
        grip_ctrl = .8
        action = [0., 0., 0., 0.]
        try:
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
            elif key.char == 'n':
                action = [0., 0., 0., grip_ctrl]
            elif key.char == 'm':
                action = [0., 0., 0., -grip_ctrl]
        except AttributeError as e:
            pass
        env.step(action)
        env.render()

    while True:
        st = time.time()
        env.unwrapped.num_obst = np.random.randint(4)
        env.reset()
        env.render()
        time.sleep(0.5)
        # for i in range(10):
        #     env.step([0., 0., 0., 0.])
        #     env.render()
        #     time.sleep(0.1)
        # print('Time:', time.time()-st)

    # Collect events until released
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


if __name__ == '__main__':
    test_env()

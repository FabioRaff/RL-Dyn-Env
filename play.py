from common import get_args
from envs.config import register_custom_envs, make_env
from envs.custom_scenarios import scenarios
from algorithm import create_agent
import os
import numpy as np
import time


class Player:
    def __init__(self, args):
        # initialize environment
        self.args = args

        self.info = []
        self.test_rollouts = 20

        self.agent = create_agent(args)
        self.agent.load(os.path.join(args.model_path, "saved_policy-{}".format(args.play_epoch)))

    def play(self):
        # play policy on env
        env = make_env(self.args)
        if self.args.scenario:
            env.scenario = scenarios[self.args.scenario]
        else:
            env.unwrapped.obj_range = 1.0
            env.unwrapped.target_range = 1.0
            env.unwrapped.num_obst = 3

        acc_sum = 0
        col_sum = 0
        seed = 700

        for i in range(self.test_rollouts):

            obs, _ = env.reset(seed=seed + i)
            print('Seed: ', seed + i)

            for timestep in range(args.timesteps):
                action = self.agent.step(obs)
                obs, _, _, _, info = env.step(action)

                env.render()
                # time.sleep(0.05)
                if info['Success']:
                    acc_sum += 1
                    col_sum += info['Collisions']
                    break
        print('AccSum: ', acc_sum)
        print('Collisions: ', col_sum)


if __name__ == "__main__":
    register_custom_envs()
    args = get_args()

    player = Player(args)
    player.play()

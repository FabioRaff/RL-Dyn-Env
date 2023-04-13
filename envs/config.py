import gymnasium as gym

Robotics_envs_id = [
    'RandDynObstEnv-v1'
]


def register_custom_envs():
    gym.envs.register(
        id='RandDynObstEnv-v1',
        entry_point='envs:RandDynObstEnv',
        max_episode_steps=100,
        kwargs={'render_mode': 'human'}
    )


def make_env(args):
    return gym.make(
        args.env,
        n_substeps=args.env_n_substeps,
        obj_lost_reward=args.obj_lost_reward,
        collision_reward=args.collision_reward,
        num_obst=args.num_obst)

def make_vector_env(args):
    return gym.vector.make(
        args.env,
        n_substeps=args.env_n_substeps,
        num_envs=args.num_envs,
        obj_lost_reward=args.obj_lost_reward,
        collision_reward=args.collision_reward,
        num_obst=args.num_obst)


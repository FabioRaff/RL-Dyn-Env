# RL-Dyn-Env

Trained on Ubuntu 20.04
Python Version: 3.7

train robot with random generated obstacles:

`train.py`

the model will be saved under newly created `log` directory.

Play  model in mujoco:

`play.py --model_path path/to/model/dir`

fine tune on custom scenario (e.g. lifted_obstacles):

`train.py --model_path path/to/model/dir --scenario lifted_obst`







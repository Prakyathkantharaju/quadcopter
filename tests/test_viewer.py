import numpy as np
from dm_control import viewer
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from env_utils import test_env_viewer


env = test_env_viewer()
action_spec = env.action_spec()

def policy(time):
    action = [10,-10,10,-10]
    # print(dir(env))
    return action

viewer.launch(env, policy=policy)


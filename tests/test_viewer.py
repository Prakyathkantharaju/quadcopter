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
    action = np.random.uniform(low=action_spec.minimum,
                        high=action_spec.maximum,
                        size=action_spec.shape)
    return action

viewer.launch(env, policy=policy)


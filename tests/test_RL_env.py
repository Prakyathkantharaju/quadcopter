import numpy as np
from dm_control import viewer
import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from env_utils import make_env


env = make_env()
action_spec = env.action_spec()

# Get the `action_spec` describing the control inputs.
action_spec = env.action_spec()

# Step through the environment for one episode with random actions.
obs, _ = env.reset()

for i in range(100):
    action = env.action_space.sample()

    env.step(action)

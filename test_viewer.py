import numpy as np
from dm_control import viewer
from env_utils import test_env_viewer


env = test_env_viewer()
action_spec = env.action_spec()

def policy(time):
    action = np.random.uniform(low=action_spec.minimum,
                        high=action_spec.maximum,
                        size=action_spec.shape)
    return action

viewer.launch(env, policy=policy)


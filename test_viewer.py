
from dm_control import viewer
from env_utils import test_env_viewer


env = test_env_viewer()

viewer.launch(env)


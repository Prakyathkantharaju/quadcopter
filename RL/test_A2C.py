import numpy as np
from dm_control import viewer
import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from env_utils import make_env
from stable_baselines3 import A2C


env = make_env()
action_spec = env.action_spec()
obs,_ = env.reset()
act = env.action_space.sample()
print(act)
print(env.step(act))
print(len(obs))
print(env.observation_space)

model = A2C("MlpPolicy", env, verbose=1)

model.learn(total_timesteps=1000000, log_interval=3)

model.save("hover_A2C")


obs = env.reset()
while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render("human")

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs, deterministic=True)
#     obs, reward, done, info = env.step(action)
#     env.render("human")
#     if done:
#       obs = env.reset()
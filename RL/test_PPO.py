import numpy as np
from dm_control import viewer
import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from env_utils import make_env
from stable_baselines3 import PPO


env = make_env()
action_spec = env.action_spec()
obs,_ = env.reset()
act = env.action_space.sample()
print(act)
print(env.step(act))
print(len(obs))
print(env.observation_space)

model = PPO("MlpPolicy", env, verbose=1)

# model.learn(total_timesteps=1000000, log_interval=3)

# model.save("hover_PPO")

# Used post learning
position = []
orientation = []
action_history = [] 

model.load("RL/hover_PPO")

obs,_ = env.reset()
position.append(obs[3:6])
orientation.append(obs[:3])
# while True:
for i in range(2000):
    action, _states = model.predict(obs)
    action_history.append(action)
    obs, rewards, dones,_, info = env.step(action)
    env.render("human")
    print(action)
    position.append(obs[3:6])
    orientation.append(obs[:3])
    # if dones:
        # break

import matplotlib.pyplot as plt

plt.figure()
plt.plot(position)
plt.title('position')

plt.figure()
plt.plot(orientation)
plt.title("orientation")


plt.figure()
plt.plot(action_history)
plt.title("action")


plt.show()

# obs = env.reset()
# while True:
#     action, _states = model.predict(obs, deterministic=True)
#     obs, reward, done, info = env.step(action)
#     env.render("human")
#     if done:
#       obs = env.reset()
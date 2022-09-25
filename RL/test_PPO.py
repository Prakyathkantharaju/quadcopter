import numpy as np
from dm_control import viewer
import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from env_utils import make_env
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy


env = make_env()
action_spec = env.action_spec()
obs,_ = env.reset()
act = env.action_space.sample()
print(act)
print(env.step(act))
print(len(obs))
print(env.observation_space)

model = PPO("MlpPolicy", env, verbose=1)
# reload last policy
# model.load("RL/hover_PPO")

model.learn(total_timesteps=1000000, log_interval=3)

model.save("hover_PPO")

# Used post learning
position = []
orientation = []
action_history = [] 
# model.load("RL/hover_PPO")

obs,_ = env.reset()
# position.append(obs[3:6])
# orientation.append(obs[:3])
# while True:
for i in range(1000):
    action, _states = model.predict(obs)
    # action = env.action_space.sample()
    action_history.append(action)
    obs, rewards, dones,_, info = env.step(action)
    env.render("human")
    print(dones)
    # position.append(obs[3:6])
    # orientation.append(obs[:3])
    if dones:
        break

import matplotlib.pyplot as plt

# plt.figure()
# plt.plot(position)
# plt.title('position')

# plt.figure()
# plt.plot(orientation)
# plt.title("orientation")


plt.figure()
plt.plot(action_history)
plt.title("action")


plt.show()

print(evaluate_policy(model, env, n_eval_episodes=100))
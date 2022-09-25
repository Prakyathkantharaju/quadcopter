import numpy as np
from dm_control import viewer
import os, sys
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))
from env_utils import make_env
from stable_baselines3 import PPO
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.vec_env import DummyVecEnv

# not working
if __name__ == "__main__":
    env = DummyVecEnv([ make_env for i in range (5)])

    eval_env = make_env()

    model = PPO("MlpPolicy", env, verbose=1)
    # reload last policy
    # model.load("RL/hover_PPO")

    model.learn(total_timesteps=1000000, log_interval=1, eval_env=eval_env)

    model.save("hover_PPO_3")

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
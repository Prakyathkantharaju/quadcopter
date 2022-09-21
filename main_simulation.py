
from fnmatch import translate
import numpy as np
import matplotlib.pyplot as plt
from env_utils import make_env
from controller.controller import Controller

env = make_env()
obs,_ = env.reset() 
position = obs[3:6]
orientation = obs[:3]
K_z = np.array([1.5, 2.5])
K_psi = np.array([6, 1.75])
K_theta = np.array([6, 1.75])
K_omega = np.array([6, 1.75])
g = 9.81
m = 0.2
l = 0.1
k = 2e-5
b = 1e-7
ctrl = Controller(K_z=K_z, K_psi=K_psi, K_theta=K_theta, K_omega=K_omega, k=k, l=l, b=b)

desired_action = np.array([1, 0 , 10, 0, 10, 0, 10, 0])
for i in range(100):
    prev_position, prev_orientation = position, orientation
    obs,_,_,_,_ = env.step(env.action_space.sample())
    position, orientation = obs[3:6], obs[:3]
    # print(env.TimeStep)
    ang_vel = (orientation - prev_orientation) / 0.03
    vel = (position - prev_position) / 0.03
    print(f"orientatin: {obs[:3]}, position: {obs[3:6]}, ang_vel: {ang_vel}, vel: {vel}")
    ang = np.hstack((orientation.reshape(-1, 1), ang_vel.reshape(-1,1)))
    translation = np.hstack((position.reshape(-1,1), vel.reshape(-1,1)))
    # ctrl.get_action(desired_action=desired_action, ang=ang, translation=translation)
    env.render(mode="human")



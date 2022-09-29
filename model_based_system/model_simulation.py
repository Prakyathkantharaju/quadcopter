from model_workinprogress import LinAccel, AngAccel, Animation
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.integrate import odeint
import sys
sys.path.append('C:\\UIC courses\\ME 510 - Robotic Manipulators course\\Project\\quadcopter-main\\controller')
print(sys.path)
from controller import Controller


pause = 0.01
fps = 30
l = 0.225  # in m
k = 2.980*1e-6  # this is to be found via calculation
b = 1.140*1e-7  # this is to be found via calculation
Ixx = 4.856*1e-3
Iyy = 4.856*1e-3
Izz = 8.801*1e-3
m = 0.468  # in kg
g = 9.81  # in m/s**2
i = 0
Ax = 0.25
Ay = 0.25
Az = 0.25
# I = np.array([Ixx, 0, 0],
#              [0, Iyy, 0],
#              [0, 0, Izz])
I = np.array([Ixx, Iyy, Izz])
A = np.array([Ax, Ay, Az])
# t = Torque(l, k, b)
omega = np.array([600, 600, 600, 600])
omega = np.expand_dims(omega, axis=1)

t = np.linspace(0, 1, 101)
# T_b = t(w)
#
x0, y0, z0, vx0, vy0, vz0 = [0, 0, 0, 0, 0, 0]
theta0, psi0, phi0, thetadot0, psidot0, phidot0 = [0, 0, 0, 0, 0, 0]
X_lin_0 = np.array([x0, y0, z0, vx0, vy0, vz0], dtype='float64')
X_ang_0 = np.array([theta0, psi0, phi0, thetadot0, psidot0, phidot0], dtype='float64')  # Initial values
X_0 = np.concatenate([X_lin_0, X_ang_0])

lin = LinAccel(m, k, g, i)
a = AngAccel(I)
# xddot = np.concatenate((lin_acc, ang_acc), axis=None)  # This is a 6 x 1 vector giving the accelerations of the system
# xddot = np.expand_dims(xddot, axis=1)
# print(xddot)

parms_ang = (omega, )
X_ang = odeint(a.angular_acceleration, X_ang_0, t, args=parms_ang)
assert X_ang.shape == (len(t), 6), f'The angular acceleration should be in len(t) x 6 shape'

parms_lin = (X_ang, omega, A, t)
X_pos = odeint(lin.linear_acceleration, X_lin_0, t, args=parms_lin)
assert X_pos.shape == (len(t), 6), f'The linear acceleration should be in len(t) x 6 shape'

# fig = plt.figure()
anim = Animation(pause, fps, m, k, g, l, b, i)
anim.animate(t, X_pos, X_ang)


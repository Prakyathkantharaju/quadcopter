# Author: Subramanian - Added the code that simulates the drone using Python Matplotlib animation - 09/29/2022
# Author: Subramanian - Updated the code that involves hover control and simulates it using Matplotlib - 10/13/2022
# Author: Subramanian - Fixed the bug and stabilization controller is working - 10/14/2022

from model import LinAccel, AngAccel, Animation
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.integrate import odeint
import sys
from controller import Controller


pause = 0.005
fps = 10
l = 0.225  # in m
k = 2.980*1e-6  # this is to be found via calculation
b_drag_const = 1.140e-7  # this is to be found via calculation
Ixx = 4.856*1e-3
Iyy = 4.856*1e-3
Izz = 8.801*1e-3
m = 0.468  # in kg
g = 9.81  # in m/s**2
# A = 0.25  # Considering Ax = Ay = Az
I = np.array([Ixx, Iyy, Izz])
K_z = np.array([1.5, 2.5])
K_psi = np.array([6, 1.75])
K_theta = np.array([6, 1.75])
K_phi = np.array([-6, -1.75])
Ax = 0.25
Ay = 0.25
Az = 0.25
# I = np.array([Ixx, 0, 0],
#              [0, Iyy, 0],
#              [0, 0, Izz])
A = np.array([Ax, Ay, Az])
# t = Torque(l, k, b)
omega0 = np.array([600, 600, 600, 600])
omega0 = np.expand_dims(omega0, axis=1)
# T_b = t(w)


lin = LinAccel(m, k, g)
angacc = AngAccel(I, l, k, b_drag_const)



# Desired trajectory: Lemniscate
h = 0.05
t0 = 0
tN = 6
N = int((tN-t0)/h) + 1
t = np.linspace(t0, tN, N)
T = t[N-1]

x_0 = 0
y_0 = 0
A_const = 0.5
B = 0.5
a = 2
b = 1
mpi = np.pi
phi = np.zeros(N)
phidot = np.zeros(N)
theta = np.zeros(N)
thetadot = np.zeros(N)
psi = np.zeros(N)
psidot = np.zeros(N)

z_ref = np.zeros(N)

v_z = np.zeros(N)

a_z = np.zeros(N)

x0 = 0
y0 = 0
z0 = 1
vx0 = 0
vy0 = 0
vz0 = 0
# theta0, psi0, phi0, thetadot0, psidot0, phidot0 = [0, 0, 0, 0, 0, 0]
theta0, psi0, phi0, thetadot0, psidot0, phidot0 = [np.deg2rad(10), np.deg2rad(10), np.deg2rad(10), 0, 0, 0]
X_lin_0 = np.array([x0, y0, z0, vx0, vy0, vz0], dtype='float64')
X_ang_0 = np.array([theta0, psi0, phi0, thetadot0, psidot0, phidot0], dtype='float64')

omega = omega0
X_POS = np.zeros((len(t), 6))
X_ANG = np.zeros((len(t), 6))
X_POS[0, 0] = X_lin_0[0]
X_POS[0, 1] = X_lin_0[1]
X_POS[0, 2] = X_lin_0[2]
X_POS[0, 3] = X_lin_0[3]
X_POS[0, 4] = X_lin_0[4]
X_POS[0, 5] = X_lin_0[5]

X_ANG[0, 0] = X_ang_0[0]
X_ANG[0, 1] = X_ang_0[1]
X_ANG[0, 2] = X_ang_0[2]
X_ANG[0, 3] = X_ang_0[3]
X_ANG[0, 4] = X_ang_0[4]
X_ANG[0, 5] = X_ang_0[5]

OMEGA = np.zeros((len(t), 4))
OMEGA[0] = omega0.reshape(4, )
DES_STATE = np.zeros((len(t), 8))

for i in range(0,N-1):
    # desired_traj_values = np.array([x_ref[i], y_ref[i], z_ref[i], v_x[i], v_y[i], v_z[i], a_x[i], a_y[i], a_z[i], j_x[i], j_y[i], j_z[i], phi[i], phidot[i], phiddot[i],
    #                                 phitdot[i]])
    control = Controller(K_z, K_psi, K_theta, K_phi, A, k, l, b_drag_const)
    t_temp = np.array([t[i], t[i+1]], dtype='float64')
    # desired_state = control.get_desired_positions(t_temp, desired_traj_values)
    desired_state = np.array([z_ref[i], v_z[i], theta[i], thetadot[i], psi[i], psidot[i], phi[i], phidot[i]])
    parms_ang = (omega, )
    X_ang = odeint(angacc.angular_acceleration, X_ang_0, t_temp, args=parms_ang)
    assert X_ang.shape == (2, 6), f'The angular acceleration should be in 1 x 6 shape'
    parms_lin = (X_ang[1], omega, A)
    X_pos = odeint(lin.linear_acceleration, X_lin_0, t_temp, args=parms_lin)
    assert X_pos.shape == (2, 6), f'The linear acceleration should be in 1 x 6 shape'
    ang = np.array([[X_ang[1][0], X_ang[1][3]], [X_ang[1][1], X_ang[1][4]], [X_ang[1][2], X_ang[1][5]]], dtype='float64')
    translation = np.array([[X_pos[1][0], X_pos[1][3]], [X_pos[1][1], X_pos[1][4]], [X_pos[1][2], X_pos[1][5]]], dtype='float64')
    omega = control.get_action(desired_state, ang, translation)
    omega_temp = omega.reshape(4,)
    OMEGA[i+1] = omega_temp
    DES_STATE[i] = desired_state
    X_ang_0 = X_ang[1]
    X_lin_0 = X_pos[1]
    # X_POS.append(X_pos[1])
    # X_ANG.append(X_ang[1])
    X_POS[i+1, 0] = X_pos[1][0]
    X_POS[i+1, 1] = X_pos[1][1]
    X_POS[i+1, 2] = X_pos[1][2]
    X_POS[i+1, 3] = X_pos[1][3]
    X_POS[i+1, 4] = X_pos[1][4]
    X_POS[i+1, 5] = X_pos[1][5]

    X_ANG[i+1, 0] = X_ang[1][0]
    X_ANG[i+1, 1] = X_ang[1][1]
    X_ANG[i+1, 2] = X_ang[1][2]
    X_ANG[i+1, 3] = X_ang[1][3]
    X_ANG[i+1, 4] = X_ang[1][4]
    X_ANG[i+1, 5] = X_ang[1][5]


anim = Animation(pause, fps, m, k, g, l, b_drag_const)
anim.animate(t, X_POS, X_ANG)

fig = plt.figure()
ax = plt.subplot(111)

plt.plot(t, X_POS[:, 0:3])
plt.legend(['x_pos', 'y_pos', 'z_pos'])

fig2 = plt.figure()
ax2 = plt.subplot(111)
plt.plot(t, X_ANG[:, 0:3])
plt.legend(['Angle theta', 'Angle psi', 'Angle phi'])
# plt.plot(t, X_POS)
# plt.plot(t, X_POS)
# plt.plot(t, phitdot)

plt.show()



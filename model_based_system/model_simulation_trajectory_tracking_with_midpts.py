"""This module performs the simulation of low-level quadcopter control. The simulation
involves trajectory tracking of quadcopter right from its lift off from the ground position.
The control involves tracking two different trajectories: Lift off + tracking a path.
"""

from model import LinAccel, AngAccel, Animation
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy.integrate import odeint
import sys
from controller import Controller


pause = 0.005
fps = 0.5
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
K_psi = np.array([3, 0.75])
K_theta = np.array([3, 0.75])
K_phi = np.array([-3, -0.75])
# circle
# Kp = np.array([1.85*5, 7.55, 1.85*6])
# Kd = np.array([0.75*15, 0.75*15, 0.75*15])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5*5, 1.5*5, 1.5*5])

# lemniscate
Kp = np.array([10, 10, 10])
Kd = np.array([7, 7, 7])
Kdd = np.array([1.00, 1.00, 1.00])
Ki = np.array([1.5, 1.5, 1.5])
# Kp = np.array([1.85*5, 7.55, 1.85*5])
# Kd = np.array([0.75*10, 0.75*10, 0.75*5])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5, 1.5, 1.5])

# sine curve
# Kp = np.array([1.87*4.5, 10.55, 1.87*4.5])
# Kd = np.array([0.75*15, 0.75*15, 0.75*15])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5*5, 1.5*5, 1.5*5])

Ax = 0.25
Ay = 0.25
Az = 0.25
# I = np.array([Ixx, 0, 0],
#              [0, Iyy, 0],
#              [0, 0, Izz])
A = np.array([Ax, Ay, Az])
# t = Torque(l, k, b)
omega0 = np.array([620, 620, 620, 620])
omega0 = np.expand_dims(omega0, axis=1)

lin = LinAccel(m, k, g)
angacc = AngAccel(I, l, k, b_drag_const)


# Desired trajectory: Lift off + Lemniscate (Doing the midpoint trajectory)
h = 0.005
t0 = 0
t1 = 25
tN = 100
# N = int((tN-t0)/h) + 1
# t = np.linspace(t0, tN, N)
N1 = int((t1-t0)/h)
N2 = int((tN-t1)/h) + 1
t_lift = np.linspace(t0, t1, N1)
t_traj = np.linspace(t1, tN, N2)
T = t_traj[N2-1]

a10 = 0
a11 = 0
a12 = 0
a13 = 1280/T**3
a14 = -7680/T**4
a15 = 12288/T**5
z_ref1 = a10 + a11*t_lift + a12*t_lift**2 + a13*t_lift**3 + a14*t_lift**4 + a15*t_lift**5
z_refdot1 = a11 + 2*a12*t_lift + 3*a13*t_lift**2 + 4*a14*t_lift**3 + 5*a15*t_lift**4
z_refddot1 = 2*a12 + 2*3*a13*t_lift + 3*4*a14*t_lift**2 + 4*5*a15*t_lift**3
z_reftdot1 = a12 + 2*3*a13 + 2*3*4*a14*t_lift + 3*4*5*a15*t_lift**2

# x_0 = 0
# y_0 = 0.5
A_const = 0.5
B = 0.5
a = 2
b = 1
mpi = np.pi
# theta = np.zeros(N)
# thetadot = np.zeros(N)
# psi = np.zeros(N)
# psidot = np.zeros(N)

tau = 2*mpi*((-47/81)+(640/81)*(t_traj/T)-(3200/81)*(t_traj/T)**2+(7040/81)*(t_traj/T)**3-(6400/81)*(t_traj/T)**4+(2048/81)*(t_traj/T)**5)
taudot = 2*mpi*((640/81)*(1/T)-2*(3200/81)*(1/T)*(t_traj/T)+(7040/81)*3*(1/T)*(t_traj/T)**2-(6400/81)*4*(1/T)*(t_traj/T)**3+(2048/81)*5*(1/T)*(t_traj/T)**4)
tauddot = 2*mpi*(-2*(3200/81)*(1/T)**2+(7040/81)*3*2*(1/T)**2*(t_traj/T)-(6400/81)*4*3*(1/T)**2*(t_traj/T)**2 + (2048/81)*5*4*(1/T)**2*(t_traj/T)**3)
tautdot = 2*mpi*((7040/81)*3*2*(1/T)**3-(6400/81)*4*3*2*(1/T)**3*(t_traj/T) + (2048/81)*5*4*3*(1/T)**3*(t_traj/T)**2)

t = np.concatenate((t_lift, t_traj))
N = N1 + N2

# tau = 2*mpi*(-15*(t_traj/T)**4+6*(t_traj/T)**5+10*(t_traj/T)**3)
# taudot = 2*mpi*(-15*4*(1/T)*(t_traj/T)**3+6*5*(1/T)*(t_traj/T)**4+10*3*(1/T)*(t_traj/T)**2)
# tauddot = 2*mpi*(-15*4*3*(1/T)**2*(t_traj/T)**2 + 6*5*4*(1/T)**2*(t_traj/T)**3+10*3*2*(1/T)**2*(t_traj/T))
# tautdot = 2*mpi*(-15*4*3*2*(1/T)**3*(t_traj/T) + 6*5*4*3*(1/T)**3*(t_traj/T)**2 + 10*3*2*(1/T)**3)
# taujdot = 2*mpi*(-15*4*3*2*(1/T)**3 + 6*5*4*3*2*(1/T)**4*(t/T))

'''# Trajectory: Lemniscate'''
# Gains that give beter tracking:
# Kp = np.array([1.85*5, 7.55, 1.85*5])
# Kd = np.array([0.75*10, 0.75*10, 0.75*10])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5*5, 1.5*5, 1.5*5])

# Another set of gains for good trajectory tracking:
# Kp = np.array([1.85*5, 7.55, 1.85*5])
# Kd = np.array([0.75*10, 0.75*10, 0.75*5])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5, 1.5, 1.5])

phi = np.zeros(N)
phidot = np.zeros(N)
phiddot = np.zeros(N)
phitdot = np.zeros(N)

x_ref1 = np.zeros(N1)+0.5
y_ref1 = np.zeros(N1)
z_ref1 = z_ref1
v_x1 = np.zeros(N1)
v_y1 = np.zeros(N1)
v_z1 = z_reftdot1
a_x1 = np.zeros(N1)
a_y1 = np.zeros(N1)
a_z1 = z_refddot1
j_x1 = np.zeros(N1)
j_y1 = np.zeros(N1)
j_z1 = z_reftdot1

x_ref2 = B*np.cos(b*tau)
y_ref2 = A_const*np.sin(a*tau)
z_ref2 = np.zeros(N2)+2
v_x2 = -B*b*np.sin(b*tau)*taudot
v_y2 = A_const*a*np.cos(a*tau)*taudot
v_z2 = np.zeros(N2)
a_x2 = -B*b*b*np.sin(b*tau)*taudot-B*b*np.sin(b*tau)*tauddot
a_y2 = -A_const*a*a*np.sin(a*tau)*taudot+A_const*a*np.cos(a*tau)*tauddot
a_z2 = np.zeros(N2)
j_x2 = -B*b*b*b*np.cos(b*tau)*taudot-B*b*b*np.sin(b*tau)*tauddot - B*b*b*np.cos(b*tau)*tauddot-B*b*np.sin(b*tau)*tautdot
j_y2 = -A_const*a*a*a*np.cos(a*tau)*taudot-A_const*a*a*np.sin(a*tau)*tauddot - A_const*a*a*np.sin(a*tau)*tauddot + A_const*a*np.cos(a*tau)*tautdot
j_z2 = np.zeros(N2)
# joun_x = A_const*a*a*a*a*np.sin(a*tau)*taudot - A_const*a*a*a*np.cos(a*tau)*tauddot - A_const*a*a*a*np.cos(a*tau)*tauddot - A_const*a*a*np.sin(a*tau)*tautdot - A_const*a*a*a*np.cos(a*tau)*tauddot - A_const*a*a*np.sin(a*tau)*tautdot - A_const*a*a*np.sin(a*tau)*tautdot + A_const*a*np.cos(a*tau)*taujdot
# joun_y = B*b*b*b*b*np.sin(b*tau)*taudot - -B*b*b*b*np.cos(b*tau)*tauddot - B*b*b*b*np.cos(b*tau)*tauddot - B*b*b*np.sin(b*tau)*tautdot + B*b*b*b*np.sin(b*tau)*tauddot - B*b*b*np.cos(b*tau)*tautdot - B*b*b*np.cos(b*tau)*tautdot - B*b*np.sin(b*tau)*taujdot
# joun_z = np.zeros(N)

x_ref = np.concatenate((x_ref1, x_ref2))
y_ref = np.concatenate((y_ref1, y_ref2))
z_ref = np.concatenate((z_ref1, z_ref2))
v_x = np.concatenate((v_x1, v_x2))
v_y = np.concatenate((v_y1, v_y2))
v_z = np.concatenate((v_z1, v_z2))
a_x = np.concatenate((a_x1, a_x2))
a_y = np.concatenate((a_y1, a_y2))
a_z = np.concatenate((a_z1, a_z2))
j_x = np.concatenate((j_x1, j_x2))
j_y = np.concatenate((j_y1, j_y2))
j_z = np.concatenate((j_z1, j_z2))

'''# Trajectory: Circle'''
# Gains that gives better tracking:

# Kp = np.array([1.85*4, 8.55, 1.85*4])
# Kd = np.array([0.75*15, 0.75*15, 0.75*15])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5*5, 1.5*5, 1.5*5])

# Another set of good gains:

# Kp = np.array([1.85*5, 7.55, 1.85*6])
# Kd = np.array([0.75*15, 0.75*15, 0.75*15])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5*5, 1.5*5, 1.5*5])

# x_ref = A_const*np.cos(tau)
# y_ref = A_const*np.sin(tau)
# z_ref = np.zeros(N)
#
# v_x = -A_const*np.sin(tau)*taudot
# v_y = A_const*np.cos(tau)*taudot
# v_z = np.zeros(N)
# a_x = -A_const*np.sin(tau)*tauddot - A_const*a*np.cos(tau)*taudot
# a_y = -A_const*np.cos(tau)*tauddot-A_const*np.sin(tau)*taudot
# a_z = np.zeros(N)
# j_x = np.zeros(N)
# j_y = np.zeros(N)
# j_z = np.zeros(N)

'''# Trajectory: sine curve'''
# Gains that gives better tracking:

# Kp = np.array([1.87*4.5, 10.55, 1.87*4.5])
# Kd = np.array([0.75*15, 0.75*15, 0.75*15])
# Kdd = np.array([1.00, 1.00, 1.00])
# Ki = np.array([1.5*5, 1.5*5, 1.5*5])

# x_ref = tau
# y_ref = np.sin(tau)
# z_ref = np.zeros(N)
#
# v_x = taudot
# v_y = np.cos(tau)*taudot
# v_z = np.zeros(N)
# a_x = tauddot
# a_y = -np.sin(tau)*taudot + np.cos(tau)*tauddot
# a_z = np.zeros(N)
# j_x = np.zeros(N)
# j_y = np.zeros(N)
# j_z = np.zeros(N)

'''# Trajectory: Straight line'''
# x1 = 0
# y1 = 0
# r = 2
# x_ref = x1 + r*np.cos(tau)
# y_ref = y1 + r*np.sin(tau)
# z_ref = np.zeros(N)
#
# v_x = -r*np.sin(tau)*taudot
# v_y = r*np.cos(tau)*taudot
# v_z = np.zeros(N)
# a_x = -r*np.sin(tau)*tauddot - r*np.cos(tau)*taudot
# a_y = r*np.cos(tau)*tauddot - r*np.sin(tau)*taudot
# a_z = np.zeros(N)
# j_x = np.zeros(N)
# j_y = np.zeros(N)
# j_z = np.zeros(N)

x0 = x_ref[0]
y0 = y_ref[0]
z0 = z_ref[0]
vx0 = 0
vy0 = 0
vz0 = 0
theta0, psi0, phi0, thetadot0, psidot0, phidot0 = [0, 0, 0, 0, 0, 0]
# theta0, psi0, phi0, thetadot0, psidot0, phidot0 = [np.deg2rad(10), np.deg2rad(10), np.deg2rad(10), 0, 0, 0]
X_lin_0 = np.array([x0, y0, z0, vx0, vy0, vz0], dtype='float64')
X_ang_0 = np.array([theta0, psi0, phi0, thetadot0, psidot0, phidot0], dtype='float64')
lin_acc_prev = np.array([0, 0, 0], dtype='float64')

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
DES_STATE = np.zeros((len(t), 9))
THRUST = [0]
T_THETA = [0]
T_PSI = [0]
T_PHI = [0]
THETADOT = [0]
PSIDOT = [0]
THETA = [0]
PSI = [0]
error_x = [0]
error_y = [0]

for i in range(0,N-1):
    desired_traj_values = np.array([x_ref[i], y_ref[i], z_ref[i], v_x[i], v_y[i], v_z[i], a_x[i], a_y[i], a_z[i], j_x[i], j_y[i], j_z[i], phi[i], phidot[i], phiddot[i],
                                    phitdot[i]])
    t_temp = np.array([t[i], t[i+1]], dtype='float64')
    linacc, jerk = lin.linear_acceleration_duplicate(X_lin_0, t_temp, X_ang_0, omega, A, lin_acc_prev)
    actual_traj_values = np.array([X_lin_0[0], X_lin_0[1], X_lin_0[2], X_lin_0[3], X_lin_0[4], X_lin_0[5], linacc[0], linacc[1], linacc[2], jerk[0], jerk[1], jerk[2]])
    control = Controller(K_z, K_psi, K_theta, K_phi, Kp, Kd, Kdd, Ki, A, k, l, b_drag_const)
    # desired_state, thetadot, psidot = control.get_desired_positions(t_temp, desired_traj_values, actual_traj_values)
    desired_state = control.get_desired_positions(t_temp, desired_traj_values, actual_traj_values)
    parms_ang = (omega, )
    X_ang = odeint(angacc.angular_acceleration, X_ang_0, t_temp, args=parms_ang)
    assert X_ang.shape == (2, 6), f'The angular acceleration should be in 1 x 6 shape'
    parms_lin = (X_ang[1], omega, A)
    X_pos = odeint(lin.linear_acceleration, X_lin_0, t_temp, args=parms_lin)
    assert X_pos.shape == (2, 6), f'The linear acceleration should be in 1 x 6 shape'
    ang = np.array([[X_ang[1][0], X_ang[1][3]], [X_ang[1][1], X_ang[1][4]], [X_ang[1][2], X_ang[1][5]]], dtype='float64')
    translation = np.array([[X_pos[1][0], X_pos[1][3]], [X_pos[1][1], X_pos[1][4]], [X_pos[1][2], X_pos[1][5]]], dtype='float64')
    vertical = translation[2]
    torques, theta, psi = control._get_torques(vertical, ang, desired_state)
    THRUST.append(torques[0])
    T_THETA.append(torques[1])
    T_PSI.append(torques[2])
    T_PHI.append(torques[3])
    THETA.append(theta)
    PSI.append(psi)
    omega = control.get_action(desired_state, ang, translation)
    omega_temp = omega.reshape(4,)
    OMEGA[i+1] = omega_temp
    DES_STATE[i] = desired_state
    X_ang_0 = X_ang[1]
    X_lin_0 = X_pos[1]
    lin_acc_prev = linacc.reshape(len(linacc), )
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
    error_x.append(x_ref[i] - X_pos[1][0])
    error_y.append(y_ref[i] - X_pos[1][1])


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

fig3 = plt.figure()
ax3 = plt.subplot(111)
plt.plot(t, THRUST)
plt.plot(t, T_THETA)
plt.plot(t, T_PSI)
plt.plot(t, T_PHI)
plt.legend(['thrust', 't_theta', 't_psi', 't_phi'])

fig4 = plt.figure()
ax4 = plt.subplot(111)
plt.plot(X_POS[:,0], X_POS[:,1], color='black', marker='o', markersize=2)
plt.plot(x_ref, y_ref, color='blue', linestyle='-')
ax4.set_aspect('equal')

fig5 = plt.figure()
ax5 = plt.subplot(411)
plt.plot(t_traj, tau)
plt.subplot(412)
plt.plot(t_traj, taudot)
plt.subplot(413)
plt.plot(t_traj, tauddot)
plt.subplot(414)
plt.plot(t_traj, tautdot)

fig6 = plt.figure()
ax6 = plt.subplot(111)
plt.plot(X_ANG[:,0], X_ANG[:,1])
plt.plot(THETA, PSI)
plt.legend(['act', 'desired'])

fig7 = plt.figure()
ax7 = plt.subplot(211)
plt.plot(t, np.zeros(len(t)), color='black', linestyle='--')
plt.plot(t, error_x, color='blue')
plt.ylabel('x_error')
plt.subplot(212)
plt.plot(t, np.zeros(len(t)), color='black', linestyle='--')
plt.plot(t, error_y, color='blue')
plt.ylabel('y_error')
plt.xlabel('time')


plt.show()

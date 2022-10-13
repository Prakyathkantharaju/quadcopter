# Author prakyath 09/19/2022
# Subramanian - Completed the angular acceleration code section and included Linear acceleration EOMs - 09/19/2022
# Subramanian - Tested the code and fixed the bugs - 09/21/2022
# Subramanian - Modified animation code lines (Tested and working). Just simulation of drone falling - 09/29/2022
# Subramanian - Fixed some bugs in Coriolis force function - 10/13/2022
from typing import Any
import numpy as np
import numpy.typing as npt
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from scipy import interpolate

# Given motor speeds (omega), and angles (theta, psi, phi) -> calculate the lin acceleration in inertial frame


class LinAccel(object):
    def __init__(self, m: float, k: float, g: float) -> None:
        """
        Class to calculate the linear acceleration. Ref. to either equation 10 or 15
        :param m: mass of the quadcopter
        """
        self._m = m
        self._k = k
        self._g = g

    def Rotation_matrix(self, X_ang: npt.ArrayLike) -> np.ndarray:
        """
        Get the rotation matrix for transformation from body frame to inertial frame
        :param ang: [theta, psi, phi] in radians
        :return: numpy array
        """
        ang = np.array([X_ang[0], X_ang[1], X_ang[2]])
        S_theta = np.sin(ang[0])
        S_psi = np.sin(ang[1])
        S_phi = np.sin(ang[2])
        C_theta = np.cos(ang[0])
        C_psi = np.cos(ang[1])
        C_phi = np.cos(ang[2])
        R = np.array([[C_phi * C_psi, (C_phi * S_psi * S_theta) - (S_phi * C_theta),
                       (C_phi * S_psi * C_theta) + (S_phi * S_theta)],
                      [S_phi * C_psi, (S_phi * S_psi * S_theta) + (C_phi * C_theta),
                       (S_phi * S_psi * C_theta) - (C_phi * S_theta)],
                      [-S_psi, C_psi * S_theta, C_psi * C_theta]])
        R = np.squeeze(R)

        return R

    def linear_acceleration(self, X_lin: npt.ArrayLike, time, X_ang: npt.ArrayLike, w: npt.ArrayLike, A: npt.ArrayLike) -> np.ndarray:
        """
        Calculate Linear accelerations equation 10 or 15 in PDF
        :param w: quadrotor motor speeds : Size = 4 x 1
        :param X_ang: All the angular positions and angular velocities that's been integrated across the time steps
        from 0 to T. Shape is : (101, 6) for T = 101. The elements in
        each row of X_ang are: [theta, psi, phi, theta_dot, psi_dot, phi_dot]
        :return: Derivative of linear vector : Shape = (6, 1). The elements are derivatives of 'X_lin'.
        """
        # X_lin = np.array([X0[0], X0[1], X0[2], X0[3], X0[4], X0[5]])
        # X_ang = np.array([X0[6], X0[7], X0[8], X0[9], X0[10], X0[11]])
        Ax = A[0]
        Ay = A[1]
        Az = A[2]
        lin_vel = np.array([X_lin[3], X_lin[4], X_lin[5]], dtype='float64')
        lin_vel = lin_vel.reshape(3, 1)
        # lin_vel = np.expand_dims(lin_vel, axis=1)
        ang = np.array([X_ang[0], X_ang[1], X_ang[2]], dtype='float64')
        ang = ang.reshape(3, 1)
        w1 = w[0]
        w2 = w[1]
        w3 = w[2]
        w4 = w[3]
        T = np.array([0, 0, self._k * (w1**2 + w2**2 + w3**2 + w4**2)], dtype='float64')
        T = T.reshape(3, 1)
        G = np.array([0, 0, -self._g])
        G = G.reshape(3, 1)
        R = self.Rotation_matrix(ang)
        thrust = R.dot(T) / self._m
        drag_coeffs = np.array([[Ax, 0, 0],
                                [0, Ay, 0],
                                [0, 0, Az]], dtype='float64')
        drag_dyn = drag_coeffs.dot(lin_vel)
        drag_dyn = drag_dyn.reshape(3, 1)
        drag = drag_dyn / self._m
        lin_acc = G + thrust - drag
        # lin_acc = np.expand_dims(lin_acc, axis=1)
        lin_dot = np.concatenate((lin_vel, lin_acc))
        lin_dot = lin_dot.reshape(6,)
        # self.i += 1
        # if self.i == len(X_ang):
        #     self.i -= 1
        return lin_dot


# Given the theta, psi and phi -> angular accel in body frame.



class Torque:
    def __init__(self) -> None:
        """
        Class to get torque given the angular velocity of rotor

        Inputs
        l: float -> length between the COM and the fin.
        k: float -> motor dimensional constant
        b: float -> damping dimensional constant
        """
        # l = 0.225  # in m
        # k = 2.980e-6  # this is to be found via calculation
        # b = 1.140e-7  # this is to be found via calculation
        self._l = l
        self._k = k
        self._b = b

    def __call__(self, w: npt.ArrayLike) -> np.ndarray:
        """
        Get the Torque vector
        w: motor angular velocity in shape  (4,1)
        """
        assert w.shape == (4, 1), f"The omega shape should be (4,1), currently it is {w.shape}"
        w1 = w[0]
        w2 = w[1]
        w3 = w[2]
        w4 = w[3]
        T_b = np.array([self._l * self._k * (-w2**2 + w4**2),
                        self._l * self._k * (-w1**2 + w3**2),
                        self._b * (w1**2 - w2**2 + w3**2 - w4**2)], dtype='float64')
        return T_b.reshape(3, 1)


class AngAccel(Torque):
    def __init__(self, I, l, k, b) -> None:
    """
    Calculate angular accelerations equation (20) in the pdf.
    given:
        Inertia matrix: (1x3)
    """
    super().__init__(l, k, b)
    # l = 0.225  # in m
    # k = 2.980e-6  # this is to be found via calculation
    # b = 1.140e-7  # this is to be found via calculation
    self.I = I
    self._l = l
    self._k = k
    self._b = b

    def Jacobian(self, X_ang: npt.ArrayLike) -> np.ndarray:
        """
        Calculate jacobian
        ang -> [theta, psi, phi] in radian
        """
        ang = np.array([X_ang[0], X_ang[1], X_ang[2]])
        I = self.I
        S = np.sin
        C = np.cos
        J = np.array([[I[0], 0, -I[0] * S(ang[1])],
                      [0, I[1] * C(ang[0]) ** 2 + I[2] * S(ang[0]) ** 2,
                       (I[1] - I[2]) * C(ang[0]) * S(ang[0]) * C(ang[1])],
                      [-I[0] * S(ang[1]), (I[1] - I[2]) * C(ang[0]) * S(ang[0]) * C(ang[1]),
                       I[0] * S(ang[1]) ** 2 + I[1] * S(ang[0]) ** 2 * C(ang[1]) ** 2 + I[2] * C(ang[0]) ** 2 * C(ang[1]) ** 2]])

        assert J.shape == (3, 3), f"jacobian is not in correct shape"

        return J

    def Coroilis_force(self, X_ang: npt.ArrayLike) -> np.ndarray:
        """
        Coriolis matrix
        Input:
        :param X_ang: All the angular positions and angular velocities. The elements in
         it are: [theta, psi, phi, theta_dot, psi_dot, phi_dot] in radians and rad/s
        """
        ang = np.array([X_ang[0], X_ang[1], X_ang[2]])
        vel = np.array([X_ang[3], X_ang[4], X_ang[5]])
        S_theta = np.sin(ang[0])
        S_psi = np.sin(ang[1])
        S_phi = np.sin(ang[2])
        C_theta = np.cos(ang[0])
        C_psi = np.cos(ang[1])
        C_phi = np.cos(ang[2])
        I = self.I

        # getting indiviual terms
        C_11 = 0
        C_12 = ((I[1] - I[2]) * ((vel[1] * C_theta * S_theta) + (vel[2] * S_theta ** 2 * C_psi))) + ((I[2] - I[1]) * vel[2] *
               C_theta ** 2 * C_psi) - (I[0] * vel[2] * C_psi)
        C_13 = (I[2] - I[1]) * vel[2] * C_theta * S_theta * C_psi ** 2
        C_21 = ((I[2] - I[1]) * (vel[1] * C_theta * S_theta + vel[2] * S_theta * C_psi)) + ((I[1] - I[2]) * vel[2] *
               C_theta ** 2 * C_psi) + (I[0] * vel[2] * C_psi)
        C_22 = (I[2] - I[1]) * vel[0] * C_theta * S_theta
        C_23 = (-I[0] * vel[2] * S_psi * C_psi) + (I[1] * vel[2] * (S_theta ** 2) * S_psi * C_psi) + (I[2] * vel[2] * (
                    C_theta ** 2) * S_psi * C_psi)
        C_31 = ((I[1] - I[2]) * vel[2] * C_theta * S_theta * (C_psi ** 2)) - (I[0] * vel[1] * C_psi)
        C_32 = ((I[2] - I[1]) * ((vel[1] * C_theta * S_theta * S_psi) + (vel[0] * S_theta ** 2 * C_psi))) + (
                    (I[1] - I[2]) * vel[0] * (C_theta ** 2) * C_psi) + (I[0] * vel[2] * S_psi * C_psi) - (I[1] * vel[2] * \
               (S_theta ** 2) * S_psi * C_psi) - (I[2] * vel[2] * (C_theta ** 2) * S_psi * C_psi)
        C_33 = ((I[1] - I[2]) * vel[0] * C_theta * S_theta * (C_psi ** 2)) - (I[1] * vel[1] * (
                    S_theta ** 2) * C_psi * S_psi) - (I[2] * vel[1] * (C_theta ** 2) * C_psi * S_psi) + (I[0] * vel[1] * C_psi * S_psi)

        C = np.array([[C_11, C_12, C_13],
                      [C_21, C_22, C_23],
                      [C_31, C_32, C_33]])

        return C
    
    def angular_acceleration(self, X_ang: npt.ArrayLike, time, w: npt.ArrayLike) -> np.ndarray:
        """
        Calculates the angular acceleration
        :param X_ang: All the angular positions and angular velocities. The elements in
         it are: [theta, psi, phi, theta_dot, psi_dot, phi_dot] in radians and rad/s
        :param time: time steps to integrate
        :param w: omega (motor speeds of the quadcopter). Shape is : (4, 1)
        :return: Derivative of angular values with shape = (6, 1). The elements are the derivative of 'X_ang'.
        """
        ang = np.array([X_ang[0], X_ang[1], X_ang[2]])
        vel = np.array([X_ang[3], X_ang[4], X_ang[5]])
        t = Torque(self._l, self._k, self._b)
        T_b = t(w)
        T_b = np.squeeze(T_b)

        J = self.Jacobian(ang)
        C = self.Coroilis_force(X_ang)
        diff = T_b - (C.dot(vel))
        Jinv = np.linalg.inv(J)
        ang_acc = Jinv.dot(diff)
        # ang_acc = np.expand_dims(ang_acc, axis=1)
        ang_dot = np.concatenate((vel, ang_acc))
        return ang_dot

    
 class Animation(LinAccel):
    def __init__(self, pause: float, fps: float, m: float, k: float, g: float, l: float, b: float) -> None:
        """
        This class does the animation of a quadcopter in 3D environment. Linear Acceleration class is inherited to get
        get the rotation matrix details.
        :param pause: To pause the animation
        :param fps: number of frames per second for animation
        :param m: mass of the quadcopter
        :param k: lift constant
        :param g: acceleration due to gravity
        :param l: length between COM and the fins/axles
        :param b: drag/damping dimensional constant
        """
        super().__init__(m, k, g)
        self.pause = pause
        self.fps = fps
        self.m = m
        self.k = k
        self.g = g
        self.l = l
        self.b = b


    def animate(self, t: npt.ArrayLike, Xpos: npt.ArrayLike, Xang: npt.ArrayLike) -> None:
        """
        This function animates the drone simulation.
        :param t: integration time step
        :param Xpos: Gets the 3-D linear positions of the drone: [x, y, z]
        :param Xang: Gets the 3-D angular positions of the drone: [theta, psi, phi] in radians
        :return: The animation of drone hovering
        """
        t_interp = np.arange(t[0],t[len(t)-1], 1 / self.fps)
        [m, n] = np.shape(Xpos)
        shape = (len(t_interp), n)
        Xpos_interp = np.zeros(shape)
        Xang_interp = np.zeros(shape)
        l = self.l

        for i in range(0, n):
            fpos = interpolate.interp1d(t, Xpos[:,i])
            Xpos_interp[:,i] = fpos(t_interp)
            fang = interpolate.interp1d(t, Xang[:,i])
            Xang_interp[:,i] = fang(t_interp)


        axle_x = np.array([[-l/2, 0, 0],
                           [l/2, 0, 0]])
        axle_y = np.array([[0, -l/2,  0],
                           [0, l/2,   0]])


        [p2,q2] = np.shape(axle_x)

        for ii in range(0,len(t_interp)):
            x = Xpos_interp[ii,0]
            y = Xpos_interp[ii,1]
            z = Xpos_interp[ii,2]
            theta = Xang_interp[ii,0]
            psi = Xang_interp[ii,1]
            phi = Xang_interp[ii,2]
            ang = np.array([theta, psi, phi])
            R = self.Rotation_matrix(ang)

            new_axle_x = np.zeros((p2,q2))
            for i in range(0,p2):
                r_body = axle_x[i,:]
                r_world = R.dot(r_body)
                new_axle_x[i, :] = r_world

            new_axle_x = np.array([x, y, z]) + new_axle_x
            # print(new_axle_x)

            new_axle_y = np.zeros((p2,q2))
            for i in range(0,p2):
                r_body = axle_y[i,:]
                r_world = R.dot(r_body)
                new_axle_y[i, :] = r_world

            new_axle_y = np.array([x, y, z]) + new_axle_y
            # print(new_axle_y)

            ax = p3.Axes3D(fig)
            axle1, = ax.plot(new_axle_x[:, 0],new_axle_x[:, 1],new_axle_x[:, 2], 'ro-', linewidth=3)
            axle2, = ax.plot(new_axle_y[:, 0], new_axle_y[:, 1], new_axle_y[:, 2], 'bo-', linewidth=3)

            ax.set_xlim(-2, 2)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)
            ax.view_init(azim=-72, elev=20)

            plt.pause(self.pause)

        plt.close()
        
fig = plt.figure()

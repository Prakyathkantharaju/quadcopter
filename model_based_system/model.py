# Author prakyath 09/19/2022
# Subramanian - Completed the angular acceleration code section and included Linear acceleration EOMs - 09/19/2022
from typing import Any
import numpy as np
import numpy.typing as npt

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

    def Rotation_matrix(self, ang: npt.ArrayLike) -> np.ndarray:
        """
        Get the rotation matrix for transformation from body frame to inertial frame
        :param ang: [theta, psi, phi] in radians
        :return: numpy array
        """
        S_theta = np.sin(ang[0])
        S_psi = np.sin(ang[1])
        S_phi = np.sin(ang[2])
        C_theta = np.cos(ang[0])
        C_psi = np.cos(ang[1])
        C_phi = np.cos(ang[2])
        R = np.array([[C_phi*C_psi, (C_phi*S_psi*S_theta)-(S_phi*C_theta), (C_phi*S_psi*C_theta)+(S_phi*S_theta)],
                      [S_phi*C_psi, (S_phi*S_psi*S_theta)+(C_phi*C_theta), (S_phi*S_psi*C_theta)-(C_phi*S_theta)],
                      [-S_psi, C_psi*S_theta, C_psi*C_theta]])
        R = np.squeeze(R)

        return R

    def linear_acceleration(self, w: npt.ArrayLike, ang: npt.ArrayLike) -> np.ndarray:
        """
        Calculate Linear accelerations equation 10 or 15 in PDF
        :param w: quadrotor motor speeds
        :param ang: angles -> [theta, psi, phi] in radians
        :return: linear acceleration vector : Shape = 3 x 1
        """
        w **= 2
        T = np.array([0, 0, self._k*(w[0] + w[1] + w[2] + w[3])], dtype=object)
        G = np.array([0, 0, -self._g]) / self._m
        R = self.Rotation_matrix(ang)
        thrust = R.dot(T) / self._m
        lin_acc = G + thrust

        return lin_acc


# Given the theta, psi and phi -> angular accel in body frame.



class Torque:

    def __init__(self, l: float, k: float, b: float) -> None:
        """
        Class to get torque given the angular velocity of rotor

        # TODO add more details here
        Inputs
        l: float -> length between the COM and the fin.
        k: float -> motor dimensional constant
        b: float -> damping dimensional constant
        """
        self._l = l
        self._k = k
        self._b = b

    def __call__(self, w: npt.ArrayLike) -> np.ndarray:
        """
        Get the Torque vector
        w: motor angular velocity in shape  (4,1)
        """
        assert w.shape == (4,1), f"The omega shape should be (4,1), currently it is {w.shape}"
        w **= 2
        T_b = np.array([self._l * self._k * (-w[1] + w[3]),
                        self._l * self._k * (-w[0] + w[4]),
                        self._b * (w[0] - w[1] + w[2] - w[3])])
        return T_b.reshape(3, 1)


class AngAccel(Torque):
    def __init__(self, I: npt.ArrayLike, ) -> None:
        """
        Calculate angular accelerations equation (20) in the pdf.
        given:
            Inertia matrix: (1x3)
        """
        super().__init__(l, k, b)
        self.I = I
        self._l = l
        self._k = k
        self._b = b

    def Jacobian(self, ang: npt.ArrayLike) -> np.ndarray:
        """
        Calculate jacobian
        ang -> [theta, psi, phi] in radian
        """
        I = self.I
        S = np.sin
        C = np.cos
        J = np.array([[I[0], 0, -I[0]*S(ang[0])],
                     [0, I[1]*C(ang[1])**2 + I[2]*S(ang[1])**2, (I[1] - I[2])*C(ang[1])*S(ang[1])*C(ang[0])],
                     [-I[0] * S(ang[0]), (I[1] - I[2])*C(ang[1])*S(ang[1])*C(ang[0]), I[0] * S(ang[0])**2 + I[1] * S(ang[1])**2 * C(ang[0])**2 + I[2] * C(ang[1])**2 * C(ang[0])**2]])

        assert J.shape == (3, 3), f"jacobian is not in correct shape"

        return J

    def Coroilis_force(self, ang: npt.ArrayLike, vel: npt.ArrayLike) -> np.ndarray:
        """
        Coriolis matrix
        Input:
            ang -> [theta, psi, omega] in radian
            vel: ang_vel -> [theta_dot, psi_dot, omega_dot] in radian/s
        """
        S_theta = np.sin(ang[0])
        S_psi = np.sin(ang[1])
        S_phi = np.sin(ang[2])
        C_theta = np.cos(ang[0])
        C_psi = np.cos(ang[1])
        C_phi = np.cos(ang[2])
        I = self.I

        # getting indiviual terms
        C_11 = 0
        C_12 = (I[1] - I[2]) * (vel[0] * C_theta * S_psi + vel[1] * S_psi**2 * C_theta) + (I[2] - I[1]) * vel[1] * C_psi**2 * C_theta - I[0]*vel[1]*C_theta
        C_13 = (I[2] - I[1]) * vel[1] * C_psi * S_psi * C_theta ** 2
        C_21 = (I[2] - I[1]) * (vel[0] * C_psi * S_psi + vel[1] * S_psi * C_theta) + (I[1] - I[2]) * vel[1] * C_psi**2 * C_theta + I[0]
        C_22 = (I[3] - I[2]) * vel[2] * C_phi * S_phi
        C_23 = -I[0] * vel[1]*S_theta*C_theta + I[1]*vel[1]*(S_phi ** 2)*S_theta*C_theta + I[2]*vel[1]*(C_phi ** 2)*S_theta*C_theta
        C_31 = ((I[1] - I[2])*vel[2]*C_phi*S_phi*(C_theta ** 2)) - I[0]*vel[0]*C_theta
        C_32 = (I[2] - I[1])*vel[2]*C_phi*S_phi*(C_theta ** 2) + ((I[1] - I[2])*vel[2]*(C_phi ** 2)*C_theta) + I[0]*vel[0]*S_theta*C_theta - I[1]*vel[0]*(S_phi ** 2)*S_theta*C_theta - I[2]*vel[0]*(C_phi ** 2)*S_theta*C_theta
        C_33 = (I[1] - I[2])*vel[2]*C_phi*S_phi*(C_theta ** 2) - I[1]*vel[0]*(S_phi ** 2)*C_theta*S_theta - I[1]*vel[0]*(S_phi ** 2)*C_theta*S_theta - I[2]*vel[0]*(C_phi ** 2)*C_theta*S_theta + I[0]*vel[0]*C_theta*S_theta

        C = np.array([[C_11, C_12, C_13],
                      [C_21, C_22, C_23],
                      [C_31, C_32, C_33]])

        return C

    def angular_acceleration(self, w: npt.ArrayLike, ang: npt.ArrayLike, vel: npt.ArrayLike) -> np.ndarray:
        t = Torque(self._l, self._k, self._b)
        T_b = t(w)
        T_b = np.squeeze(T_b)

        J = self.Jacobian(ang)
        C = self.Coroilis_force(ang, vel)
        diff = T_b - np.matmul(C, vel)
        Jinv = np.linalg.inv(J)
        ang_acc = Jinv.dot(diff)

        return ang_acc



# Assuming some values for sake of completeness of the code. For our purpose, we should figure that out for the quadcopter
l = 0.225   # in m
k = 2.980e-6
b = 1.140e-7
Ixx = 4.856e-3
Iyy = 4.856e-3
Izz = 8.801e-3
m = 0.468  # in kg
g = 9.81  # in m/s**2
# I = np.array([Ixx, 0, 0],
#              [0, Iyy, 0],
#              [0, 0, Izz])
I = np.array([Ixx, Iyy, Izz])
# t = Torque(l, k, b)
omega = np.zeros((4, 1))
# T_b = t(w)
#
lin = LinAccel(m, k, g)

ang = np.array([1, 0.5, 2.7])  # some random values
ang_vel = np.array([1, 1, 1])  # some random values

lin_acc = lin.linear_acceleration(omega, ang)

assert lin_acc.shape == (3, 1), f'The linear acceleration should be in 3 x 1 shape'

a = AngAccel(I)
# J = a.Jacobian(ang)
# C = a.Coroilis_force(ang, ang_vel)

"""Computing the differential equations for the angular accelerations (eqn. 20 from the PDF)"""
# diff = T_b - np.matmul(C, ang_vel)
# Jinv = np.linalg.inv(J)
ang_acc = a.angular_acceleration(omega, ang, ang_vel)

assert ang_acc.shape == (3, 1), f'The angular acceleration should be in 3 x 1 shape'


xddot = np.concatenate((lin_acc, ang_acc), axis=None)  # This is a 6 x 1 vector giving the accelerations of the system



# TODO: Low-level PD controller


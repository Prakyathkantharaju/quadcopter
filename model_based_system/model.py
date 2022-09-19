# Author prakyath 09/19/2022
from typing import Any
import numpy as np
import numpy.typing as npt





# Given the theta, psi and omega -> angular accel in body frame.



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
        T_b = np.array([self.l * self.k * (-w[1] + w[3]),
                        self.l * self.k * (-w[0] + w[4]),
                        self.b * (w[0] - w[1] + w[2] - w[3])])
        return T_b.reshape(3,1)


    


class AngAccel(object):
    def __init__(self, I: npt.ArrayLike, ) -> None:
        """
        Calculate angular accelerations equation (20) in the pdf.
        given:
            Inertia matrix: (1x3)
        """
        self.I = I

    def Jacobian(self, ang: npt.ArrayLike) -> np.ndarray:
        """
        Calculate jacobian
        ang -> [theta, psi, omega] in radian
        """
        I = self.I
        S = np.sin
        C = np.cos
        J = np.array([[I[0], 0, -I[0]*S(ang[0])],
                    [0, I[1]*C(ang[1])**2 + I[2]*S(ang[1])**2, (I[1] - I[2])*C(ang[1])*S(ang[1])*C(ang[0])],
                    [-I[0] * S(ang[0]), (I[1] - I[2])*C(ang[1])*S(ang[1])*C(ang[0]), I[0] * S(ang[0])**2 + I[1] * S(ang[1])**2 * C(ang[0])**2 + I[2] * C(ang[1])**2 * C(ang[0])**2]])

        assert J.shape == (3,3), f"jacobian is not in correct shape"

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
        C_theta = np.cos(ang[0])
        C_psi = np.cos(ang[1])
        I = self.I

        # getting indiviual terms
        C_11 = 0
        C_12 = (I[1] - I[2]) * (vel[0] * C_theta * S_psi + vel[1] * S_psi**2 * C_theta) + (I[2] - I[1]) * vel[1] C_psi**2 * C_theta - I[0]*vel[1]*C_theta
        C_13 = (I[2] - I[1]) * vel[1] * C_psi * S_psi * C_theta ** 2
        C_21 = (I[2] - I[1]) * (vel[0] * C_psi * S_psi + vel[1] * S_psi * C_theta) + (I[1] - I[2]) * vel[1] * C_psi**2 * C_theta + I[0]


        
        
        




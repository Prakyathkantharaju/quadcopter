import numpy as np
import numpy.typing as npt



class Controller:
    def __init__(self, K_z: npt.ArrayLike, K_psi: npt.ArrayLike, K_theta: npt.ArrayLike, K_omega: npt.ArrayLike, k: float,
                l: float, b: float) -> None:
        self.K_z = K_z
        self.K_psi = K_psi
        self.K_theta = K_theta
        self.K_omega = K_omega
        self.k = k
        self.l = l
        self.b = b


        # Constants
        self.g = 9.8
        self.m = 0.2
        self.I = np.array([4.8e-3, 4.8e-3, 8.8e-3])

    def _get_torques(self, vertical: npt.ArrayLike, ang: npt.ArrayLike, desired_state:npt.ArrayLike) -> np.ndarray:
        """Get the torque given the vertical, ang and desired_state

        Args:
            vertical (npt.ArrayLike): shape (2,1), 0 -> position, 1 -> velocity
            ang (npt.ArrayLike): 3x2, 0 -> [theta dot(theta)] 1-> [psi dot(psi)], 2-> [omega dot(omega)]
            desired_state (npt.ArrayLike): [z_hat z_hat_dot psi_hat psi_hat_dot theta theta_hat_dot 
                                            omega_hat omega_hat_dot]

        Returns:
            np.ndarray: [T, T_psi, T_theta, T_omega]
        """
        # "Getting torque for given angle and torques"

        # calculating the elevation torque.
        accel = self.g + self.K_z[1]*(vertical[1] - desired_state[1]) + self.K_z[0] *(vertical[0] - desired_state[0])
        T = accel * self.m / (np.cos(ang[1,0]) * np.cos(ang[2,0]))

        T_psi = (self.K_psi[1] * (desired_state[3] - ang[0,1]) + self.K_psi[0] * (desired_state[2] - ang[0,0])) * self.I[0]
        T_theta = (self.K_theta[1] * (desired_state[5] - ang[1,1]) + self.K_theta[0] * (desired_state[4] - ang[1,0])) * self.I[1]
        T_omega = (self.K_omega[1] * (desired_state[7] - ang[2,1]) + self.K_omega[0] * (desired_state[6] - ang[2,0])) * self.I[2]


        return np.array([T, T_psi, T_theta, T_omega])

    def get_action(self, desired_action: npt.ArrayLike, ang: npt.ArrayLike, translation: npt.ArrayLike) -> np.ndarray:
        """Get the control action given desired and ang, translation

        Args:
            desired_action (npt.ArrayLike): Shape (7) [z_hat, z_hat_dot, psi_hat, psi_hat_dot, theta_hat, theta_hat_dot,
            omega_hat, omega_hat_dot]
            ang (npt.ArrayLike): 3x2 0-> psi psi_dot, 1-> theta, theta_dot, 2->omega, omega_dot
            translation (npt.ArrayLike): 3x2 position and velocity array

        Returns:
            np.ndarray: 4x1 [w_1, w_2, w_3, w_4]
        """

        T = self._get_torques(vertical=translation[2,:].reshape(2,1),ang=ang, desired_state=desired_action)

        w_1 = np.sqrt(T[0] / (4 * self.k) - T[2] / (2 * self.k * self.l) - T[3] / (4 * self.b))
        w_2 = np.sqrt(T[0] / (4 * self.k) - T[1] / (2 * self.k * self.l) + T[3] / (4 * self.b))
        w_3 = np.sqrt(T[0] / (4 * self.k) + T[2] / (2 * self.k * self.l) - T[3] / (4 * self.b))
        w_4 = np.sqrt(T[0] / (4 * self.k) + T[1] / (2 * self.k * self.l) + T[3] / (4 * self.b))

        return np.array([w_1, w_2, w_3, w_4])

        



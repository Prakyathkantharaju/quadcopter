#Subramanian - Updated the controller with a function for trajectory tracking and made slight changes in the variable names (eg. K_phi instead of K_omega) - 10/13/2022

import numpy as np
import numpy.typing as npt



class Controller:
    def __init__(self, K_z: npt.ArrayLike, K_psi: npt.ArrayLike, K_theta: npt.ArrayLike, K_phi: npt.ArrayLike, A: npt.ArrayLike, k: float,
                l: float, b: float) -> None:
        self.K_z = K_z
        self.K_psi = K_psi
        self.K_theta = K_theta
        self.K_phi = K_phi
        self.A = A[0]  # Considering Ax = Ay = Az = 0.25
        self.k = k
        self.l = l
        self.b = b
        self.last_ang_pos = np.zeros(3)

        # Constants
        self.g = 9.81
        self.m = 0.468
        self.I = np.array([4.856*1e-3, 4.856*1e-3, 8.801*1e-3])
        
    def get_desired_positions(self, t_step, des_traj_vals) -> np.ndarray:
        """
        Get the desired trajectory values for feeding them into the controller as desired values. In this case, the
        desired trajectory is Lemniscate.
        :param t_step: calculate the state of variables at step 't'
        :param des_traj_vals: desired state and its derivative values
        :return: desired trajectory values
        """
        x, y, z = des_traj_vals[0:3]
        v_x, v_y, v_z = des_traj_vals[3:6]
        a_x, a_y, a_z = des_traj_vals[6:9]
        j_x, j_y, j_z = des_traj_vals[9:12]
        phi, phidot, phiddot, phitdot = des_traj_vals[12:16]
        t = t_step[1]
        m = self.m
        g = self.g
        A = self.A
        mpi = np.pi
        # tpp = np.zeros(3)
        # S_phi = np.sin(des_ang)
        # C_phi = np.cos(des_ang)
        # dx = xddot + self.A[0]*xdot/self.m
        # dy = yddot + self.A[1]*ydot/self.m
        # dz = zddot + self.A[2]*zdot/self.m
        # tpp[0] = np.arcsin((dx*S_phi - dy*C_phi) / (dx ** 2 + dy ** 2 + (dz+self.g) ** 2))
        # tpp[1] = np.arctan(dx*C_phi + dy*S_phi / (dz + self.g))
        # tpp[2] = 0
        # desired_state = np.concatenate(z, tpp)
        theta = np.arcsin(((a_x + (A*v_x/m))*np.sin(phi) - (a_y + (A*v_y/m))*np.cos(phi)) / ((a_x + (A*v_x/m))**2 + (a_y + (A*v_y/m))**2 + (a_z + (A*v_z/m) + g)**2))
        thetadot = (((a_x + A*v_x/m)*np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) - (a_y + A*v_y/m)
                     *np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)))*(-(a_x + A*v_x/m)*(2*j_x
                     + 2*A*a_x/m) - (a_y + A*v_y/m)*(2*j_y + 2*A*a_y/m)
                     - (2*j_z + 2*A*a_z/m)*(g + a_z + A*v_z/m))/((a_x + A*v_x/m)**2
                     + (a_y + A*v_y/m)**2 + (g + a_z + A*v_z/m)**2)**2 + (mpi*(a_x + A*v_x/m)*(12*t**4/625 - 24*t**3/125 + 12*t**2/25)
                     *np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) + mpi*(a_y + A*v_y/m)*(12*t**4/625 - 24*t**3/125 + 12*t**2/25)
                     *np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) + (j_x + A*a_x/m)
                     *np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) - (j_y + A*a_y/m)
                     *np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)))/((a_x + A*v_x/m)**2 + (a_y + A*a_y/m)**2
                     + (g + a_z + A*v_z/m)**2))/np.sqrt(-((a_x + A*v_x/m)*np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25))
                     - (a_y + A*v_y/m)*np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)))**2/((a_x + A*v_x/m)**2
                     + (a_y + A*v_y/m)**2 + (g + a_z + A*v_z/m)**2)**2 + 1)

        psi = np.arctan(((a_x + (A*v_x/m))*np.cos(phi) + (a_y + (A*v_y/m))*np.sin(phi)) / ((a_x + (A*v_x/m))**2 + (a_y + (A*v_y/m))**2 + (a_z + (A*v_z/m) + g)**2))
        psidot = (((a_x + A*v_x/m)*np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) + (a_y + A*v_y/m)
                   *np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)))*(-(a_x + A*v_x/m)*(2*j_x + 2*A*a_x/m) - (a_y + A*v_y/m)*(2*j_y + 2*A*a_y/m)
                   - (2*j_z + 2*A*a_z/m)*(g + a_z + A*v_z/m))/((a_x + A*v_x/m)**2
                   + (a_y + A*v_y/m)**2 + (g + a_z + A*v_z/m)**2)**2 + (-mpi*(a_x + A*v_x/m)*(12*t**4/625 - 24*t**3/125 + 12*t**2/25)*np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) + mpi*(a_y + A*v_y/m)
                   *(12*t**4/625 - 24*t**3/125 + 12*t**2/25)*np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) + (j_x
                   + A*a_x/m)*np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)) + (j_y
                   + A*a_y/m)*np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)))/((a_x + A*v_x/m)**2
                   + (a_y + A*v_y/m)**2 + (g + a_z + A*v_z/m)**2))/(((a_x + A*v_x/m)*np.cos(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25))
                   + (a_y + A*v_y/m)*np.sin(mpi*(12*t**5/3125 - 6*t**4/125 + 4*t**3/25)))**2/((a_x + A*v_x/m)**2
                   + (a_y + A*v_y/m)**2 + (g + a_z + A*v_z/m)**2)**2 + 1)
        desired_state = np.array([z, v_z, theta, thetadot, psi, psidot, phi, phidot])
        return desired_state
        
    def _get_torques(self, vertical: npt.ArrayLike, ang: npt.ArrayLike, desired_state:npt.ArrayLike) -> np.ndarray:
        """Get the torque given the vertical, ang and desired_state
        Args:
            vertical (npt.ArrayLike): shape (2,1), 0 -> position, 1 -> velocity
            ang (npt.ArrayLike): 3x2, 0 -> [theta(theta dot)] 1-> [psi(psi dot)], 2-> [phi(phi dot)]
            desired_state (npt.ArrayLike): [z_hat z_hat_dot theta_hat theta_hat_dot psi_hat psi_hat_dot
                                            phi_hat phi_hat_dot]
        Returns:
            np.ndarray: [T, T_psi, T_theta, T_omega]
        """
        # "Getting torque for given angle and torques"

        # calculating the elevation torque.
        accel = self.g + self.K_z[1]*(desired_state[1] - vertical[1]) + self.K_z[0] * (desired_state[0] - vertical[0])
        T = accel * self.m / (np.cos(ang[0, 0]) * np.cos(ang[1, 0]))

        T_theta = (self.K_theta[1] * (desired_state[3] - ang[0,1]) + self.K_theta[0] * (desired_state[2] - ang[0,0])) * self.I[0]
        T_psi = (self.K_psi[1] * (desired_state[5] - ang[1,1]) + self.K_psi[0] * (desired_state[4] - ang[1,0])) * self.I[1]
        T_phi = (self.K_phi[1] * (desired_state[7] - ang[2,1]) + self.K_phi[0] * (desired_state[6] - ang[2,0])) * self.I[2]

        return np.array([T, T_theta, T_psi, T_phi])
    
    def get_action(self, desired_action: npt.ArrayLike, ang: npt.ArrayLike, translation: npt.ArrayLike) -> np.ndarray:
        """Get the control action given desired and ang, translation
        Args:
            desired_action (npt.ArrayLike): Shape (8) [z_hat, z_hat_dot, psi_hat, psi_hat_dot, theta_hat, theta_hat_dot,
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

        print("---------------------------------------------------------------------------")

        print(T[0] / (4 * self.k), -T[2] / (2 * self.k * self.l), -T[3] / (4 * self.b))
        print(T[0] / (4 * self.k), - T[1] / (2 * self.k * self.l), + T[3] / (4 * self.b))
        print(T[0] / (4 * self.k), T[2] / (2 * self.k * self.l), - T[3] / (4 * self.b))
        print(T[0] / (4 * self.k), T[1] / (2 * self.k * self.l), T[3] / (4 * self.b))

        print("---------------------------------------------------------------------------")

        rotor_speeds_fdbk = np.array([w_1, w_2, w_3, w_4])
        return rotor_speeds_fdbk.reshape(len(rotor_speeds_fdbk), 1)
        



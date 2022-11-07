#Subramanian - Updated the controller with a function for trajectory tracking and made slight changes in the variable names (eg. K_phi instead of K_omega) - 10/13/2022
#Subramanian - Updated the controller to contain the trajectory tracking section with adding integral term (PID control) - 10/26/2022

import numpy as np
import numpy.typing as npt


class Controller:
    def __init__(self, K_z: npt.ArrayLike, K_psi: npt.ArrayLike, K_theta: npt.ArrayLike, K_phi: npt.ArrayLike, Kp: npt.ArrayLike,
                 Kd: npt.ArrayLike, Kdd: npt.ArrayLike, Ki: npt.ArrayLike, A: npt.ArrayLike, k: float,
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
        self.integral_error_x = 0
        self.integral_error_y = 0
        self.integral_error_z = 0

        # Constants
        self.g = 9.81
        self.m = 0.468
        self.I = np.array([4.856*1e-3, 4.856*1e-3, 8.801*1e-3])
        self.Kx, self.Ky, self.Kz = Kp
        self.Kdx, self.Kdy, self.Kdz = Kd
        self.Kddx, self.Kddy, self.Kddz = Kdd
        self.Kix, self.Kiy, self.Kiz = Ki
        
    def get_desired_positions(self, t_step, des_traj_vals, act_traj_vals) -> np.ndarray:
        """
        Get the desired trajectory values for feeding them into the controller as desired values. In this case, the
        desired trajectory is Lemniscate.
        :param t_step: calculate the state of variables at step 't'
        :param des_traj_vals: desired state and its derivative values
        :param act_traj_vals: actual state and its derivative values
        :return: desired trajectory values
        """
        xd, yd, zd = des_traj_vals[0:3]
        v_xd, v_yd, v_zd = des_traj_vals[3:6]
        a_xd, a_yd, a_zd = des_traj_vals[6:9]
        j_xd, j_yd, j_zd = des_traj_vals[9:12]
        phi, phidot, phiddot, phitdot = des_traj_vals[12:16]

        x, y, z = act_traj_vals[0:3]
        y += 0.001*np.random.random(size=1)[0]
        z += + 0.01*np.random.random(size=1)[0]
        v_x, v_y, v_z = act_traj_vals[3:6]
        a_x, a_y, a_z = act_traj_vals[6:9]
        j_x, j_y, j_z = act_traj_vals[9:12]

        t = t_step[1] - t_step[0]
        m = self.m
        g = self.g
        A = self.A
        Kx, Ky, Kz = self.Kx, self.Ky, self.Kz
        Kxd, Kyd, Kzd = self.Kdx, self.Kdy, self.Kdz
        Kxdd, Kydd, Kzdd = self.Kddx, self.Kddy, self.Kddz
        Kxi, Kyi, Kzi = self.Kix, self.Kiy, self.Kiz
        mpi = np.pi
        sin = np.sin
        cos = np.cos
        ex = (xd - x)
        ey = (yd - y)
        ez = (zd - z)
        exd = (v_xd - v_x)
        eyd = (v_yd - v_y)
        ezd = (v_zd - v_z)
        exdd = (a_xd - a_x)
        eydd = (a_yd - a_y)
        ezdd = (a_zd - a_z)
        self.integral_error_x += ex * t
        self.integral_error_y += ey * t
        self.integral_error_z += ez * t
        d_x = Kx*(ex) + Kxi*self.integral_error_x + Kxd*(exd) + Kxdd*(exdd)
        d_y = Ky*(ey) + Kyi*self.integral_error_x + Kyd*(eyd) + Kydd*(eydd)
        d_z = Kz*(ez) + Kzi*self.integral_error_x + Kzd*(ezd) + Kzdd*(ezdd)
        print("************************************************")

        print("Integral error: {}, {}, {}".format(self.integral_error_x, self.integral_error_y, self.integral_error_z))
        print("Errors in PD for linear positions: {}, {}, {}".format(ex, ey, ez))
        print("Errors in PD for linear velocities: {}, {}, {}".format(exd, eyd, ezd))
        print("Errors in PD for linear accelerations: {}, {}, {}".format(exdd, eydd, ezdd))

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
        theta = np.arcsin((d_x*np.sin(phi) - d_y*np.cos(phi)) / (d_x**2 + d_y**2 + (d_z + g)**2))
        # theta = Kx*(xd - x) + Kxd*(v_xd - v_x) + Kxdd*(a_xd - a_x)
        thetadot = (((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))*sin(phi) - (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))*cos(phi))*(-(2*Kxd*(-a_x + a_xd) + 2*Kxdd*(-j_x + j_xd))*(Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd)) - (2*Kyd*(-a_y + a_yd) + 2*Kydd*(-j_y + j_yd))*(Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd)) - (2*Kzd*(-a_z + a_zd) + 2*Kzdd*(-j_z + j_zd))*(Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g))/((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))**2 + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))**2 + (Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g)**2)**2 + ((Kxd*(-a_x + a_xd) + Kxdd*(-j_x + j_xd))*sin(phi) - (Kyd*(-a_y + a_yd) + Kydd*(-j_y + j_yd))*cos(phi) + (Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))*cos(phi)*phidot + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))*sin(phi)*phidot)/((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))**2 + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))**2 + (Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g)**2))/np.sqrt(-((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))*sin(phi) - (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))*cos(phi))**2/((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))**2 + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))**2 + (Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g)**2)**2 + 1)

        psi = np.arctan(((d_x*np.cos(phi) + d_y*np.sin(phi)) / (d_x**2 + d_y**2 + (d_z + g)**2)))
        psidot = (((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))*cos(phi) + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))*sin(phi))*(-(2*Kxd*(-a_x + a_xd) + 2*Kxdd*(-j_x + j_xd))*(Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd)) - (2*Kyd*(-a_y + a_yd) + 2*Kydd*(-j_y + j_yd))*(Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd)) - (2*Kzd*(-a_z + a_zd) + 2*Kzdd*(-j_z + j_zd))*(Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g))/((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))**2 + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))**2 + (Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g)**2)**2 + ((Kxd*(-a_x + a_xd) + Kxdd*(-j_x + j_xd))*cos(phi) + (Kyd*(-a_y + a_yd) + Kydd*(-j_y + j_yd))*sin(phi) - (Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))*sin(phi)*phidot + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))*cos(phi)*phidot)/((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))**2 + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))**2 + (Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g)**2))/(((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))*cos(phi) + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))*sin(phi))**2/((Kx*(-x + xd) + Kxd*(-v_x + v_xd) + Kxdd*(-a_x + a_xd))**2 + (Ky*(-y + yd) + Kyd*(-v_y + v_yd) + Kydd*(-a_y + a_yd))**2 + (Kz*(-z + zd) + Kzd*(-v_z + v_zd) + Kzdd*(-a_z + a_zd) + g)**2)**2 + 1)

        T = m*(d_x*(sin(psi)*cos(phi)*cos(theta) + sin(phi)*sin(theta)) + d_y*(sin(psi)*sin(phi)*cos(theta) - cos(psi)*sin(theta)) + (d_z+g)*(cos(psi)*cos(theta)))
        # desired_state = np.array([zd, v_zd, theta, thetadot, psi, psidot, phi, phidot, T])
        # return desired_state, thetadot, psidot
        desired_state = np.array([zd, v_zd, theta, psi, phi, phidot, T, thetadot, psidot], dtype='float64')
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
        '''For hovering control'''
#         accel = self.g + self.K_z[1]*(desired_state[1] - vertical[1]) + self.K_z[0] * (desired_state[0] - vertical[0])
#         T = accel * self.m / (np.cos(ang[0, 0]) * np.cos(ang[1, 0]))
        '''For trajectory tracking control'''
        T = desired_state[6]

        T_theta = (self.K_theta[1] * (-ang[0,1]) + self.K_theta[0] * (desired_state[2] - ang[0,0])) * self.I[0]
        T_psi = (self.K_psi[1] * (-ang[1,1]) + self.K_psi[0] * (desired_state[3] - ang[1,0])) * self.I[1]
        T_phi = (self.K_phi[1] * ( - ang[2,1]) + self.K_phi[0] * (desired_state[4] - ang[2,0])) * self.I[2]
        # print(ang[2,1], ang[2,0])
        etheta = desired_state[2] - ang[0,0]
        epsi = desired_state[3] - ang[1,0]
        ephi = desired_state[4] - ang[2,0]
        ethetad = -ang[0,1]
        epsid = -ang[1,1]
        ephid = desired_state[5] - ang[2,1]
        print("Errors in PD for angular positions: {}, {}, {}".format(etheta, epsi, ephi))
        print("Errors in PD for angular velocities: {}, {}, {}".format(ethetad, epsid, ephid))

        print("************************************************")
        return np.array([T, T_theta, T_psi, T_phi]), desired_state[2], desired_state[3]
    
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

        # print("---------------------------------------------------------------------------")
        #
        # print(T[0] / (4 * self.k), -T[2] / (2 * self.k * self.l), -T[3] / (4 * self.b))
        # print(T[0] / (4 * self.k), - T[1] / (2 * self.k * self.l), + T[3] / (4 * self.b))
        # print(T[0] / (4 * self.k), T[2] / (2 * self.k * self.l), - T[3] / (4 * self.b))
        # print(T[0] / (4 * self.k), T[1] / (2 * self.k * self.l), T[3] / (4 * self.b))
        #
        # print("---------------------------------------------------------------------------")

        rotor_speeds_fdbk = np.array([w_1, w_2, w_3, w_4])
        return rotor_speeds_fdbk.reshape(len(rotor_speeds_fdbk), 1)
        



import numpy
from aux_fuctions import *

class ShipModel:
    def __init__(self):
        # Ship parameters
        self.T = 4.5  # draught [m]
        self.U0_knots = 10.5173753  # nominal speed [knots]
        self.U0 = self.U0_knots * 0.514444  # nominal speed [m/s]
        self.L = 137.2  # Length between perpendiculars L_pp [m]
        self.m = 4726000  # mass [Kg]
        self.B = 21  # breadth [m]
        self.Cb = 0.36  # block coefficient
        self.rho = 1025  # density of the water [Kg/m3]
        self.inertia_radius = 0.26 * self.L  # radius of gyration
        self.xg = 2  # distance from the CO to the CG along the x direction [m]
        self.Izz = self.m * self.inertia_radius ** 2 + self.m * self.xg ** 2  # moment of inertia about the CO [Kgm2]
        self.dt = 1 # time step

        # non-dimensional parameters
        self.m_ = self.m / (0.5 * self.L ** 3 * self.rho)
        self.xg_ = self.xg / self.L
        self.Izz_ = self.Izz / (0.5 * self.L ** 5 * self.rho)

        self.Xu_dot = -(2.7 * self.rho * ((self.m / self.rho) ** (5 / 3))) / (self.L ** 2)
        self.Xu_dot_ = self.Xu_dot / (0.5 * self.rho * self.L ** 3)
        self.Yv_dot_ = -np.pi * ((self.T / self.L) ** 2) * (0.67 * (self.B / self.L) - 0.0033 * (self.B / self.T) ** 2)
        self.Yr_dot_ = -np.pi * ((self.T / self.L) ** 2) * (1 + 0.16 * ((self.Cb * self.B) / (self.T)) - 5.1 * (self.B / self.L) ** 2)
        self.Nr_dot_ = -np.pi * ((self.T / self.L) ** 2) * ((1 / 12) + 0.017 * ((self.Cb * self.B) / (self.T)) - 0.33 * (self.B / self.L))
        self.Nv_dot_ = -np.pi * ((self.T / self.L) ** 2) * (1.1 * (self.B / self.L) - 0.041 * (self.B / self.T))

        # Wind parameters
        self.Cdt = 0.9
        self.CdlAF_0 = 0.45
        self.CdlAF_pi = 0.5
        self.delta = 0.8
        self.kappa = 1.1
        self.Alw = 1400
        self.Afw = 261.74
        self.Loa = 142.9
        self.sl = 6.32

        # Estimative of the hydrodynamic derivatives - estimated under no wind
        self.X_para = np.array([-0.00131102, -0.00055358, -0.00087127,  0.01070035, -0.00597487, -0.00646003,
                                -0.00109026, 0.00353707, 0.0015745, 0.00498188])
        self.Y_para = np.array([-9.38558083e-03, -4.43777544e-03, -1.28875814e-01, -8.92032884e-02, -4.58694576e-03,
                                -3.91877858e-03, -1.22278204e-04,  2.45027764e-04, -1.06104297e-05, -1.52928263e-03,
                                -1.24290109e-02,  7.86037402e-02,  1.11068554e-06,  4.90775636e-05,  6.96582498e-05])
        self.N_para = np.array([-9.05661944e-04, -5.60340137e-04, -1.74800033e-02, -1.29992765e-02, -9.20403948e-04,
                                -3.35957065e-04, -5.04495800e-05,  9.98276958e-05,  2.24600023e-04, -2.26034708e-04,
                                -2.01641383e-03,  8.75744186e-03,  2.04300980e-07,  9.48237498e-06,  1.39504335e-05])


    def predict(self, force_model, u0, v0, r0, heading0, x0, y0, d, wind_d, wind_speed):
        # Initialize lists to store state variables: nu contains the velocities and eta the heading and position of the vessel
        nu_list = [np.array([u0, v0, r0])]  # u0 is the variation of the surge speed from the nominal speed at the first time instant
        nu_U0_list = [np.array([u0 + self.U0, v0, r0])]
        eta = np.array([x0, y0, heading0])
        eta_list = [eta]

        for k in range(len(d)-1):
            # Update the position and heading based on the current velocities
            eta = eta + np.dot(rot_z(eta[2]), nu_U0_list[-1]) * self.dt

            # Calculate the magnitude of the velocity
            U_new = np.sqrt((self.U0 + nu_list[-1][0]) ** 2 + (nu_list[-1][1]) ** 2)

            # Calculate the vector of the components for the caluclation of the forces
            component_x = force_model.get_X(nu_list[-1][0], nu_list[-1][1], nu_list[-1][2], d[k], U_new, self.L)
            component_y = force_model.get_Y(nu_list[-1][0], nu_list[-1][1], nu_list[-1][2], d[k], U_new, self.L)
            component_n = force_model.get_N(nu_list[-1][0], nu_list[-1][1], nu_list[-1][2], d[k], U_new, self.L)

            # Calculate the wind forces
            tau = get_wind_forces(wind_d[k]+np.deg2rad(180), wind_speed[k], eta_list[-1][2], self.Alw, self.Afw,
                                  nu_U0_list[-1][0], nu_list[-1][1], self.delta, self.CdlAF_0,
                                  self.CdlAF_pi, self.Cdt, self.sl, self.Loa)

            # Calculate the hydrodynamic forces
            X = np.dot(np.transpose(component_x), self.X_para)
            Y = np.dot(np.transpose(component_y), self.Y_para)
            N = np.dot(np.transpose(component_n), self.N_para)

            # Update the surge and sway speeds and yaw rate
            u_next_t = (X + (tau[0]/(0.5*self.rho*self.L**2)))*(self.dt/(self.L*(self.m_-self.Xu_dot_))) + nu_list[-1][0]
            m1 = self.L*(self.m_-self.Yv_dot_)
            m2 = self.L**2 * (self.m_*self.xg_ - self.Yr_dot_)
            m3 = self.L*(self.m_*self.xg_-self.Nv_dot_)
            m4 = self.L**2*(self.Izz_-self.Nr_dot_)
            v_next_t = ((m4*(Y + (tau[1]/(0.5*self.rho*self.L**2)))-m2*(N+(tau[1]/(0.5*self.rho*self.L**2))))/
                        (m1*m4-m3*m2))*self.dt + nu_list[-1][1]
            r_next_t = ((-m3*(Y + (tau[1]/(0.5*self.rho*self.L**2)))+m1*(N+ (tau[1]/(0.5*self.rho*self.L**2))))/
                        (m1*m4-m3*m2))*self.dt + nu_list[-1][2]

            # Update the state variable lists
            nu_list.append(np.array([u_next_t, v_next_t, r_next_t]))
            nu_U0_list.append(np.array([u_next_t + self.U0, v_next_t, r_next_t]))
            eta_list.append(eta)

        return np.array(eta_list), np.array(nu_list)

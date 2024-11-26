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

        # Estimative of the hydrodynamic derivatives - estimated under 7 knots wind at 45 degrees
        self.X_para = np.array([-0.0013761 , -0.00068034, -0.00096688,  0.00515187, -0.00659082,
                                -0.01027964, -0.00114049,  0.00347134,  0.00130058,  0.00412278])
        self.Y_para = np.array([-1.82623937e-03, -1.65597953e-03, -7.87109504e-02, -6.03895575e-02,
                                5.14977635e-03, -1.23042529e-03, -2.96143325e-04,  6.23623631e-04,
                                1.17386233e-03, -3.01610354e-04, -1.00378835e-02,  8.16204052e-02,
                                -8.06832127e-07, -2.79980228e-06, -2.19752583e-06])
        self.N_para = np.array([1.15062558e-04, -1.86697638e-04, -1.26980264e-02, -9.18130323e-03,
                                2.05713998e-04, -4.76658193e-05, -7.46887848e-05,  1.60195351e-04,
                                3.99154331e-04, -8.72058778e-05, -1.77591777e-03,  1.01156830e-02,
                                -3.56176240e-08,  2.83619757e-06,  5.69129215e-06])


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

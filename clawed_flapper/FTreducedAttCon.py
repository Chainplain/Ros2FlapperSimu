###"""Finite Time Attitude Control"""
### by Chainplain 3024 Mar 16

import numpy as np
from   scipy.spatial.transform import Rotation
EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION = 0.00000000001

class Reduced_Att_Controller():
    def __init__(self):
        self.k_omega = 5
        self.k_gamma = 2
        self.k_s     = 15#10
        self.w_x     = 0.5
        self.w_y     = 2
        
        self.r       = 0.5
        self.epsilon = 0.001
        self.epsilon_gamma = 0.2
        
        
        
    def the_controller(self, Gamma, Gamma_d, angular_vel):
        d_omega_x = 0.0
        d_omega_y = 0.0
        if Gamma[2,0] >= self.epsilon_gamma:
            (d_omega_x, d_omega_y) = self.regulate_controller(Gamma, Gamma_d, angular_vel)
        else:
            (d_omega_x, d_omega_y) = self.recovering_controller(Gamma, angular_vel)
        return (d_omega_x, d_omega_y)
            
    def regulate_controller(self, Gamma, Gamma_d, angular_vel):
        Gamma_x = Gamma[0,0]
        Gamma_y = Gamma[1,0]
        Gamma_z = Gamma[2,0]
        
        Gamma_d_x = Gamma_d[0,0]
        Gamma_d_y = Gamma_d[1,0]
        Gamma_d_z = Gamma_d[2,0]
        
        angular_vel_x = angular_vel[0,0]
        angular_vel_y = angular_vel[1,0]
        angular_vel_z = angular_vel[2,0]
        
        d_Gamma_x = Gamma_y * angular_vel_z - Gamma_z * angular_vel_y
        d_Gamma_y = Gamma_z * angular_vel_x - Gamma_x * angular_vel_z
        d_Gamma_z = Gamma_x * angular_vel_y - Gamma_y * angular_vel_x
        
        s_x = Gamma_y * angular_vel_z - Gamma_z * angular_vel_y \
            + self.k_omega * self.sigma_map_s(Gamma_x - Gamma_d_x, self.r)
        s_y = Gamma_z *  angular_vel_x - Gamma_x * angular_vel_z \
             + self.k_omega * self.sigma_map_s(Gamma_y - Gamma_d_y, self.r)
        
        # + Gamma_z ** (-1) * self.k_omega * self.dot_sigma_map_s(Gamma_x - Gamma_d_x, self.r) * d_Gamma_x\
            
        d_omega_y = Gamma_z ** (-1) * d_Gamma_y * angular_vel_z \
                    - Gamma_z ** (-1) * d_Gamma_z * angular_vel_y\
                    + Gamma_z ** (-1) * self.k_omega * self.dot_sigma_map_s(Gamma_x - Gamma_d_x, self.r) * d_Gamma_x\
                    + Gamma_z ** (-1) * self.k_s * self.sigma_map_s(s_x,self.r)\
                    + Gamma_z ** (-1) * self.w_x * np.sign(s_x)
                    
        # - Gamma_z ** (-1) * self.k_omega * self.dot_sigma_map_s(Gamma_y - Gamma_d_y, self.r) * d_Gamma_y\
        d_omega_x = Gamma_z ** (-1) * d_Gamma_x * angular_vel_z \
                    - Gamma_z ** (-1) * d_Gamma_z * angular_vel_x\
                    - Gamma_z ** (-1) * self.k_omega * self.dot_sigma_map_s(Gamma_y - Gamma_d_y, self.r) * d_Gamma_y\
                    - Gamma_z ** (-1) * self.k_s * self.sigma_map_s(s_y,self.r)\
                    - Gamma_z ** (-1) * self.w_y * np.sign(s_y)
        return (d_omega_x, d_omega_y)
        
    def recovering_controller(self, Gamma, angular_vel):
        Gamma_x = Gamma[0,0]
        Gamma_y = Gamma[1,0]
        Gamma_z = Gamma[2,0]
        
        
        angular_vel_x = angular_vel[0,0]
        angular_vel_y = angular_vel[1,0]
        angular_vel_z = angular_vel[2,0]
        
        d_Gamma_x = Gamma_y * angular_vel_z - Gamma_z * angular_vel_y
        d_Gamma_y = Gamma_z * angular_vel_x - Gamma_x * angular_vel_z
        d_Gamma_z = Gamma_x * angular_vel_y - Gamma_y * angular_vel_x
        
        omega_rx = - self.k_gamma * Gamma_y / np.sqrt( Gamma_x**2 + Gamma_y**2)
        omega_ry =   self.k_gamma * Gamma_x / np.sqrt( Gamma_x**2 + Gamma_y**2)
        
        d_omega_rx = self.k_gamma * (Gamma_x * Gamma_y * d_Gamma_x + Gamma_y**2 * d_Gamma_y)\
                     / ((Gamma_x **2 + Gamma_y **2) ** 1.5)\
                     - self.k_gamma * d_Gamma_y / np.sqrt( Gamma_x **2 + Gamma_y **2 )
        d_omega_ry = - self.k_gamma * (Gamma_x**2 * d_Gamma_x + Gamma_x * Gamma_y * d_Gamma_y)\
                     / ((Gamma_x **2 + Gamma_y **2) ** 1.5)\
                     + self.k_gamma * d_Gamma_x / np.sqrt( Gamma_x **2 + Gamma_y **2 )
        
        d_omega_x = - self.k_omega / self.k_gamma * Gamma_y\
                     / np.sqrt( Gamma_x**2 + Gamma_y**2) + d_omega_rx\
                    + self.k_omega * ( omega_rx - angular_vel_x)
        d_omega_y = self.k_omega / self.k_gamma * Gamma_x\
                     / np.sqrt( Gamma_x**2 + Gamma_y**2)  + d_omega_ry\
                    + self.k_omega * ( omega_ry - angular_vel_y)            
        return (d_omega_x, d_omega_y)
    
    def sigma_map_s(self, input, r):
        if np.abs(input) >= self.epsilon: 
            signal = np.sign(input)
            module = np. power(np.abs(input), r)
            return signal * module
        else:
            return np. power( self.epsilon, r - 1 ) * input
        
    def dot_sigma_map_s(self, input, r):
        if np.abs(input) >= self.epsilon: 
            return r * np. power( np.abs(input), r-1)
        else:
            return np. power( self.epsilon, r-1)
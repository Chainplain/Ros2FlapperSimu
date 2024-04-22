###"""chainplan Observer,"""
### by Chainplain 2022/11/30
### most variable if possible are in the form of np.matrix
import numpy as np
from   scipy.spatial.transform import Rotation
from   FilterLib import Low_Pass_Second_Order_Filter as LPSF
EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION = 0.00000000001

class Finite_time_slide_mode_observer_3dim():  
    def __init__(self, robot_mass, p_init=np.mat([[0], [0], [0]]),
                 v_init=np.mat([[0], [0], [0]]), z_init=np.mat([[0], [0], [0]]),
                 time_gap=0.001, grav=9.8):
        self. p_observer = p_init
        self. v_observer = v_init
        self. z_observer = z_init
        self. rho_e = 0.5
        self. G_p = 20 * np.matrix([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]])
        self. G_v = 10 * np.matrix([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]])
        self. G_z = 5 * np.matrix([[1, 0, 0],
                               [0, 1, 0],
                               [0, 0, 1]])
        self. observer_gap = time_gap
        self. m = robot_mass
        self. e_3 = np.mat([[0], [0], [1]])
        self. gravitional_accel = grav
        
        
        
    def sigma_map(self, super, vector):
        signal = np.sign(vector)
        module = np. power(np.abs(vector), super) 
        return np.multiply(signal, module)
        
    def march_forward(self, u_t, p_real):
        # print ('self. rho_e:',self. rho_e)
        # print ('self. p_observer - p_real', self. p_observer - p_real)
        # print ('self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real)',\
                # self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real))
        p_observer_d = self. v_observer \
            - self. G_p * self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real)
        v_observer_d = 1 / self. m * u_t - self. gravitional_accel * self. e_3 \
            - 1 / self. m * self. G_v * self.sigma_map((self. rho_e + 1)/2, self. p_observer - p_real)\
            + 1 / self. m * self. z_observer
        z_observer_d = - self. G_p * self.sigma_map(self. rho_e, self. p_observer - p_real)
        
        self. p_observer = self. p_observer + p_observer_d  *  self. observer_gap
        self. v_observer = self. v_observer +  v_observer_d *  self. observer_gap
        self. z_observer = self. z_observer +  z_observer_d *  self. observer_gap

class Under_Traj_Track_Controller():
    def __init__(self, con_gap):
        self. Control_gap = con_gap
        self. K_p  = 10 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_v  = 10 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_p_inv = np.linalg.inv (self. K_p)

        self. k_tf_inv_multi_m = 10
        
        self. k_dx_in_V = 0.02 
        """
        This coefficient is based on experiment. \n 
        Corresponding to the resistance force. This coef can be set small in case of unintended behavior.
        """

        self. delta = 0.1
        """
        This coefficient is related to the hysteretic term. \n 
        No larger than 1, better set less than 0.2 .
        """

        self. k_psi = 1
        self. k_Gamma1 = 1
        self. k_Gamma2 = 0.5
        self. k_omega = 1
        self. Zero_3 = np.matrix([[0.0],[0.0],[0.0]])
        self. v_d_filter = LPSF(self. Zero_3, 8, 0.8, con_gap)
        self. psi_d_filter = LPSF(0, 8, 0.8, con_gap)
        self. omega_psi_d_filter = LPSF(0, 8, 0.8, con_gap)
        

        self. gravitional_acc = 9.8

        self. h_psi = 0
        ### Follows are the output values.
        self. f_flap_2 = 0
        self. f_flap = 0
        self. Gamma_xp = 0
        self. Gamma_yp = 0
        self. Gamma_zp = 1
        self. max_abs_Gamma_x = 0.25
        self. max_abs_Gamma_y = 0.4

    def Calc_u_t(self, p_r, p, v_r, v, psi, omega_psi ):
        e_p = p_r - p
        v_d = v_r + self. K_p * np.tanh (0.1 * e_p)
        e_v = v_d - v

        R_psi = np.mat( [[np.cos(psi), -np.sin(psi),          0],\
                         [np.sin(psi),  np.cos(psi),          0],\
                         [          0,            0,          1]])
        V_v = R_psi.T * v

        V_v_x = V_v[0, 0]
        # V_v_y = V_v[1, 0]
        # V_v_z = V_v[2, 0]

        self. v_d_filter.march_forward(v_d)
        d_v_d = self. v_d_filter.Get_filtered()
        a_d = d_v_d +self. K_v * self. K_p_inv * np.tanh (0.5 * e_p) + self. K_v * np.tanh( e_v)

        a_xd = a_d[0,0]
        a_yd = a_d[1,0]
        a_zd = a_d[2,0]

        if np.linalg.norm(e_p) < 0.2:
            psi_d = np.arctan2( v_r[1,0], v_r[0,0])
        else:
            psi_d = np.arctan2( a_yd, a_xd)
        # print('psi_d',psi_d)
        V_d_v_xd = np.sqrt( a_xd * a_xd + a_yd * a_yd)
        V_d_v_zd = a_zd

        d_v_cx = V_d_v_xd +  self. k_dx_in_V * np.sign(V_v_x) * V_v_x * V_v_x
        d_v_cz = V_d_v_zd + self. gravitional_acc

        d_v_magnitude = np.sqrt ( d_v_cx * d_v_cx + d_v_cz * d_v_cz) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION
        self.f_flap_2 = self. k_tf_inv_multi_m * d_v_magnitude
        self.f_flap = np.sqrt(self.f_flap_2)

        Gamma_xd = d_v_cx / d_v_magnitude
        if Gamma_xd > self. max_abs_Gamma_x:
            Gamma_xd = self. max_abs_Gamma_x
        
        if Gamma_xd < -0.2 * self. max_abs_Gamma_x:
            Gamma_xd = -0.2 * self. max_abs_Gamma_x

        Gamma_zd = np.sqrt(1 - Gamma_xd**2)

        delta_psi = psi_d - psi
        # print('delta_psi',delta_psi)

        if np.cos( delta_psi ) > 0:
            self. h_psi = np.sign ( np.sin( delta_psi ) )
            if self. h_psi  == 0:
                self. h_psi = 1
        
        if np.cos( delta_psi ) <= 0 and self. h_psi * np.sin( delta_psi ) <= - self. delta:
            self. h_psi = np.sign ( np.sin( delta_psi ) )
        # print('self. h_psi',self. h_psi)

        self. psi_d_filter. march_forward(psi_d)
        d_psi_d = self. psi_d_filter.Get_filtered()

        self. omega_psi_d_filter. march_forward (d_psi_d)

        omega_psi_d = self. omega_psi_d_filter.Get_filtered() + self. k_psi * self. h_psi * np. sqrt( 1 - np.cos( delta_psi))
        # print('omega_psi_d',omega_psi_d)

        Gamma_yd = self. k_Gamma1  * (  np.sign ( np.sin( delta_psi ) ) * np. sqrt( 1 - np.cos( delta_psi)) )
                #  + self. k_Gamma2 *   ( 0.5 *  self. h_psi * np. sqrt( 1 - np.cos( delta_psi)) / self. k_psi +  self. omega_psi_d_filter.Get_filtered())\
                 
        if abs(Gamma_yd) > self. max_abs_Gamma_y:
            Gamma_yd = self. max_abs_Gamma_y * np.sign(Gamma_yd)

        Gamma_magnitude = np.sqrt ( Gamma_xd * Gamma_xd + Gamma_yd * Gamma_yd + Gamma_zd * Gamma_zd) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION
        self. Gamma_xp = Gamma_xd / Gamma_magnitude 
        self. Gamma_yp = Gamma_yd / Gamma_magnitude 
        self. Gamma_zp = Gamma_zd / Gamma_magnitude 

class Simplified_Att_Controller():
    def __init__(self, con_gap):
        self. Control_gap = con_gap
        self. k_rud = 2.0
        self. k_ele = 2.0
        self. k_rud_omega = 2
        self. k_ele_omega = 2
        self. rud_omega_mag_bound = 2e-1
        self. ele_omega_mag_bound = 2e-1
        
        self. theta_rud = 0
        self. theta_ele = 0

    def Calc_u (self, Gamma_p, Gamma, Omega):
        # Gamma_p_list = list (Gamma_p_list)
        self. theta_rud = self. k_rud * ( Gamma_p[1,0] * Gamma[2,0] - Gamma_p[2,0] * Gamma[1,0] )\
                            - max(min(self. k_rud_omega * Omega[0,0], self. rud_omega_mag_bound), - self. rud_omega_mag_bound)
        
        self. theta_ele = self. k_ele * ( Gamma_p[2,0] * Gamma[0,0] - Gamma_p[0,0] * Gamma[2,0] )\
                            - max(min(self. k_ele_omega * Omega[1,0], self. ele_omega_mag_bound), - self. ele_omega_mag_bound)

class Positional_Traj_Track_Controller():
    def __init__(self, robot_mass, con_gap, g = 9.8):
    #### First controller parameters ####
        self. Control_gap = con_gap
        self. K_s = 1 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_p = 0.8 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_v  = 0.5 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. K_ev = 0.5 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_ep = 10 * np.matrix([[1,0,0],\
                                     [0,1,0],\
                                     [0,0,1]])
        self. K_ip = 0.01 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. c_s = 2
        self. rho_s = 0.5
        
        self. MaxInt = 2
        
    #### Second controller parameters ####
        self. K_ev_1 = 1 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_ep_1 = 10 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_p_1  = 0.8 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])

    #### Third controller parameters ####      
        self. K_ev_2 = 0.5 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_ep_2 = 8 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_v_2  = 0.5 * np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
        self. K_s_2 = 0.5 *    np.matrix([[1,0,0],\
                                       [0,1,0],\
                                       [0,0,1]])
                
                
        
        
        self. e_3 = np.mat([[0],[0],[1]])
        self. gravitional_accel = g
        self. m = robot_mass
        self. PIntegration =  np.matrix([[0],\
                                       [0],\
                                       [0]])
        
        
    def sigma_map(self, super, vector):
        signal = np.sign(vector)
        module = np. power(np.abs(vector), super)
        return np.multiply(signal, module)
                
    def Calc_u_t(self, p_d, p_hat, v_d, v_hat, d_v_d, z_hat, R_now):
        s = self. c_s * np. tanh(self. K_p * (p_d - p_hat) ) + self. K_v * (v_d - v_hat) 
        self. PIntegration = self. PIntegration +\
            (self. K_ip * R_now.T * (v_d - v_hat) + 3* self. K_ip * R_now.T * (p_d - p_hat)) * self. Control_gap
        # print('self. PIntegration', self. PIntegration)
        if (self. PIntegration [0,0] > self. MaxInt ):
            self. PIntegration [0,0] = self. MaxInt 
        if (self. PIntegration [0,0] < -self. MaxInt ):
            self. PIntegration [0,0] = -self. MaxInt 
            
        if (self. PIntegration [1,0] > self. MaxInt ):
            self. PIntegration [1,0] = self. MaxInt 
        if (self. PIntegration [1,0] < -self. MaxInt ):
            self. PIntegration [1,0] = -self. MaxInt 
            
        if (self. PIntegration [2,0] > self. MaxInt ):
            self. PIntegration [2,0] = self. MaxInt 
        if (self. PIntegration [2,0] < -self. MaxInt ):
            self. PIntegration [2,0] = -self. MaxInt 
        u_feedB = self. K_s * self.sigma_map(self. rho_s, s) +  self. K_ev * (v_d - v_hat)\
                 + self. K_ep * np. tanh(self. K_p * (p_d - p_hat) ) + R_now * self. PIntegration
        u_feedF = d_v_d + self. gravitional_accel * self. e_3 - 1 / self. m * z_hat 
        u_t = u_feedF + u_feedB  
        return u_t
    
    def Calc_u_t_1(self, p_d, p_hat, v_d, v_hat, d_v_d, z_hat, R_now):
        u_feedB =  self. K_ev_1 * (v_d - v_hat) + self. K_ep_1 * np. tanh(self. K_p_1 * (p_d - p_hat) ) 
        u_feedF = d_v_d + self. gravitional_accel * self. e_3
        u_t_1 = u_feedF + u_feedB
        return u_t_1
        
    def Calc_u_t_2(self, p_d, p_hat, v_d, v_hat, d_v_d, z_hat, R_now):
        s = np. sign( p_d - p_hat  + self. K_v_2 * (v_d - v_hat) )
        u_feedB =  self. K_s_2 * s + self. K_ev_1 * (v_d - v_hat) + self. K_ep_2 * ( p_d - p_hat ) 
        u_feedF = d_v_d + self. gravitional_accel * self. e_3
        u_t_2 = u_feedF + u_feedB
        return u_t_2 

def hat_map(R3vector):
        # from R^3 → a 3x3 skew-symmetric matrix
    so3matrix = np.matrix([[0.0,            -R3vector[2,0], R3vector[1,0] ],
                           [R3vector[2,0],  0.0,            -R3vector[0,0]],
                           [-R3vector[1,0], R3vector[0,0],  0.0          ]])
    return so3matrix    
        
def Computing_desired_rotation( u_t, T_d, F_d):
         u_t_normalized = 1 / (np.linalg.norm(u_t) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)\
                         * u_t
         e_3 = np. mat([[0],[0],[1]])
         
         k = hat_map( e_3 ) * u_t_normalized
         
         c_in_mat = e_3.T * u_t_normalized
         c = c_in_mat[0,0]
         
         s = np.linalg.norm(k)
         
         R_d_back_z = np.eye(3) + hat_map(k) + (1 - c) / (s * s + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION) * hat_map(k) * hat_map(k)
         
         
         T_d_normalized = 1 / (np.linalg.norm(T_d) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)\
                         * T_d
         F_d_normalized = 1 / (np.linalg.norm(F_d) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)\
                         * F_d
         # print('T_d_normalized',T_d_normalized)
         k_z =  hat_map( T_d_normalized ) * F_d_normalized 
         # print('k_z',k_z)
         c_in_mat_z = T_d_normalized.T * F_d_normalized
         c_z = c_in_mat_z[0,0]         
         
         s_z = np.linalg.norm(k_z)     
         R_z = np.eye(3) + hat_map(k_z) + (1 - c_z) / (s_z * s_z + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION) * hat_map(k_z) * hat_map(k_z)
         
         R_d = R_d_back_z * R_z
         # print('R_d',R_d)
         return R_d
         
class Attitude_reference_generator():
    def __init__(self, A_time_gap = 0.01, R_time_gap = 0.001,):
        self. R_f =     np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. Omega_f = np.matrix([[0],[0],[0]])
        self. K_wf    = 8 * np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. k_Rf    = 120
        self. G_f     =  np.matrix([[1,0,0],\
                                   [0,1,0],\
                                   [0,0,1]])
        self. generator_gap_AV = A_time_gap
        self. generator_gap_rotation = R_time_gap
                                   
    def vee_map(self, so3matrix):
        # from so(3) → R^3, well, is the inverst of the hat_map
        R3vector = np.matrix([[ 0.5 * ( so3matrix[2,1] - so3matrix[1,2]) ],
                              [ 0.5 * ( so3matrix[0,2] - so3matrix[2,0]) ],
                              [ 0.5 * ( so3matrix[1,0] - so3matrix[0,1])]])
        return R3vector
        
    def hat_map(self, R3vector):
        # from R^3 → a 3x3 skew-symmetric matrix
        so3matrix = np.matrix([[0.0,            -R3vector[2,0], R3vector[1,0] ],
                               [R3vector[2,0],  0.0,            -R3vector[0,0]],
                               [-R3vector[1,0], R3vector[0,0],  0.0          ]])
        return so3matrix
    
    def match_forward_angular_velcoity(self, R_d):
        e_R_f     = 0.5 * self. vee_map(self. G_f * R_d.T * self. R_f - self. R_f.T * R_d * self. G_f) 
        d_Omega_f = - self. K_wf * self. Omega_f - self. k_Rf * e_R_f 
        self. Omega_f = self. Omega_f + self. generator_gap_AV * d_Omega_f  
   
    def match_forward_rotation(self):
        d_R_f     = self. R_f *  self. hat_map(  self. Omega_f) 
        self. R_f = self. R_f + self. generator_gap_rotation * d_R_f 
        R_x = np.matrix([[ self. R_f[0,0]],
                         [ self. R_f[1,0]],
                         [ self. R_f[2,0]]])
        R_y = np.matrix([[ self. R_f[0,1]],
                         [ self. R_f[1,1]],
                         [ self. R_f[2,1]]])
        error_m = R_x.T * R_y
        error   = error_m[0,0]
        R_x_new = R_x - 0.5 * error * R_y
        R_x_new = R_x_new / (np.linalg.norm(R_x_new) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
        R_y_new = R_y - 0.5 * error * R_x
        R_y_new = R_y_new / (np.linalg.norm(R_y_new) + EXTREME_SMALL_NUMBER_4_ROTATION_COMPUTATION)
        R_z_new_array = np.cross(R_x_new.T, R_y_new.T)
        R_z_new = np.mat([[R_z_new_array[0,0]],[R_z_new_array[0,1]],[R_z_new_array[0,2]]])
        self. R_f = np.bmat('R_x_new, R_y_new, R_z_new')  
        
        
        # R_mid = 1 / (1 + np.trace(self. R_f )) * ( self. R_f. T  - self. R_f);
        # self. R_f = np.linalg.inv( np.eye(3) + R_mid ) * (np.eye(3) - R_mid) 
 
         
          
        
    



  
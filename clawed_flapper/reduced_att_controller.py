import rclpy
from rclpy.node import Node
from flapper_messages.msg import AngularVel
from flapper_messages.msg import UnderactuatedAttGamma
from flapper_messages.msg import Translation
from flapper_messages.msg import UnderactuatedCom

from std_msgs.msg import Float64
import numpy as np

class ReducedAttController(Node):
    def __init__(self):
        super().__init__('reduced_att_controller')
        
        # controller command publisher
        self. com_publisher = self.create_publisher(UnderactuatedCom, 'un_att_com', 1)
        self. peaker_publisher = self.create_publisher(Float64, 'peaker', 1)
        
        
        self.create_subscription(AngularVel, 'angular_vel_topic', self.angular_vel_read_callback, 1)
        self.create_subscription(UnderactuatedAttGamma, 'un_att_gamma_topic', self.underactuated_att_gamma_read_callback, 1)
        self.create_subscription(Translation, 'translation_topic', self.translation_read_callback, 1)
        
         ###------Controller Initialisation-------###
        self. Gamma_now = np.mat([0,0,1]).T
        self. Gamma_last = self. Gamma_now
        
        self. omega_now = np.mat([0,0,0]).T
        
        self. pos_now = np.mat([0.0,0.0,0.0]).T

        self. alt_int           = 0.0
        self. alt_int_mag_max   = 2.0
        self. alt_err_mag_max   = 4.0
        self. alt_vel_mag_max   = 3.0
        
        self. stroke_freq_min   = 7.0
        self. stroke_freq_max   = 15.0

        self. alt_now           = 0.0
        self. alt_last          = 0.0
        self. alt_des           = 0.0


        ###------Parameter Settings------###
        self. roll_rudder_mag_max = 0.7
        self. H_tail_mag_max      = 0.5
        
        self. time_stamp = 0.0
        self. time_stamp_last = self. time_stamp 
        
        self. time_gap = 0.01

        
        
    def angular_vel_read_callback(self, message):
        self. omega_now[0, 0] = message. x
        self. omega_now[1, 0] = message. y
        self. omega_now[2, 0] = message. z
    
    def underactuated_att_gamma_read_callback(self, message):
        # print('gamma_read')
        self. Gamma_now[0, 0] = message. x
        self. Gamma_now[1, 0] = message. y
        self. Gamma_now[2, 0] = message. z
        self. Gamma_now = self. Gamma_now / np.linalg.norm(self. Gamma_now)
    
    def translation_read_callback(self, message):
        # print('trans_read')
        self. time_stamp    = message. time_stamp
        self. pos_now[0, 0] = message. x
        self. pos_now[1, 0] = message. y
        self. pos_now[2, 0] = message. z
        self. alt_now = self. pos_now[2, 0] 
        
        peak2pubfloat = Float64()
        peak2pubfloat.data = float(self. alt_now )
        self. peaker_publisher.publish(peak2pubfloat)
        
        if (self. time_stamp  - self. time_stamp_last)  > 0:
            self. time_gap = self. time_stamp  - self. time_stamp_last 
            self. time_stamp_last = self. time_stamp 
        
        Gamma_des_x = -0.5 
        Gamma_des_y = 0 
        Gamma_des_z = np.sqrt(1 - Gamma_des_x**2 - Gamma_des_y**2)
        Gamma_des = np.mat( [Gamma_des_x, Gamma_des_y, Gamma_des_z] ).T
        Gamma_des = Gamma_des/ np.linalg.norm(Gamma_des)
        
        alt_vel_des = 0.0
        
        k_rud = 1
        k_ele = 1
        k_omega_x = 5e-2
        k_omega_y = 5e-2
        
        Gamma_now_x = self. Gamma_now[0, 0]
        Gamma_now_y = self. Gamma_now[1, 0]
        Gamma_now_z = self. Gamma_now[2, 0]
        
        
        omega_x = self. omega_now[0,0]
        omega_y = self. omega_now[1,0]
        omega_z = self. omega_now[2,0]
    
        roll_rudder_amplitude = - k_rud * (Gamma_des_y * Gamma_now_z - Gamma_des_z * Gamma_now_y) + k_omega_x * omega_x
        H_tail_amplitude = k_ele * (Gamma_des_z * Gamma_now_x - Gamma_des_x * Gamma_now_z) - k_omega_y * omega_y
        
        self. alt_des = self. alt_des + alt_vel_des * self. time_gap
        alt_vel = (self. alt_now - self. alt_last) / self. time_gap
        self. alt_last = self. alt_now
        
        k_i = 0.1
        k_p = 10
        k_d = 3
        
        e_alt = k_p * (self. alt_des - self. alt_now)
        # print('e_alt:',e_alt)
        e_alt_vel = k_d * (alt_vel_des - alt_vel)
        e_alt = max (- self. alt_err_mag_max, min(self. alt_err_mag_max, e_alt))
        self. alt_int = self. alt_int + k_i * (self. alt_des - self. alt_now)
        self. alt_int = max (- self. alt_int_mag_max, min(self. alt_int_mag_max, self. alt_int))
        
        StrokeFreq_def =  10.0
        StrokeFreq = StrokeFreq_def + self. alt_int + e_alt + e_alt_vel
        
        roll_rudder_amplitude = max( - self. roll_rudder_mag_max, \
                                    min ( roll_rudder_amplitude, self. roll_rudder_mag_max))
            
        H_tail_amplitude = max ( - self. H_tail_mag_max, \
                                    min (H_tail_amplitude, self. H_tail_mag_max))
        
        StrokeFreq = max ( self. stroke_freq_min, \
                                min (StrokeFreq, self. stroke_freq_max))
        
        com_4sub = UnderactuatedCom()
        com_4sub. time_stamp    = self. time_stamp 
        com_4sub. stroke_freq   = StrokeFreq
        com_4sub. rudder_pos    = roll_rudder_amplitude
        com_4sub. htail_pos     = H_tail_amplitude
        
        self. com_publisher.publish(com_4sub)
        
def main(args=None):
    rclpy.init(args=args)
    RAC = ReducedAttController()
    rclpy.spin(RAC)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    RAC.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

            
            


        


        
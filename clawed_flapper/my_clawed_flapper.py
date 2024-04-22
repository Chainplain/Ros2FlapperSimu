import rclpy
import numpy as np
import sys


from   lib.Bladelement import BladeAeroCalculator as BAC
from   lib.RotationComputation import GetUnitDirection_Safe
# from   lib.RotationComputation import Angle_Trajectory_Generator as ATG
from   lib.FilterLib import Low_Pass_Second_Order_Filter as LPSF
from flapper_messages.msg import Sphere
from flapper_messages.msg import AngularVel


from std_msgs.msg import String
# from rclpy.node import Node

# from   lib.FTslideControl import Finite_time_slide_mode_observer_3dim as FT_observer
# from   lib.FTslideControl import Attitude_reference_generator as ARG

# from    lib.FTreducedAttCon import Reduced_Att_Controller as RAC
# from    datetime import datetime

import os

# import random
import scipy.io as scio
from ament_index_python.packages import get_package_share_directory

# This code is made for plug-in simulation in ros2.

class ClawedFlapperDriver:
    def init(self, webots_node, properties):
        print("Starting!!")
        
        self.flapper = webots_node.robot
        print("Supervior initialized!!")
        
        
        
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_clawed_flapper')
        print("Initialization Robot node completed!!")
        
        # self.sphere_def = Sphere()
        # self.sphere_def.x = 0.0
        # self.sphere_def.y = 0.0
        # self.sphere_def.z = 0.0
        # self.sphere_def.radius = 0.0
        self. angular_vel_publisher = self.__node . create_publisher(AngularVel, 'angular_vel_topic', 10)

        package_dir = get_package_share_directory('clawed_flapper')
        Wing40X_file_path = os.path.join(package_dir, 'resource', 'SimpleFlapper0Wing40BLE_X_pos.npy')
        Wing40Y_file_path = os.path.join(package_dir, 'resource', 'SimpleFlapper0Wing40BLE_Y_pos.npy')
        Tail_V20X_file_path = os.path.join(package_dir, 'resource', 'SimpleFlapper0Tail_V20BLE_X_pos.npy')
        Tail_V20Y_file_path = os.path.join(package_dir, 'resource', 'SimpleFlapper0Tail_V20BLE_Y_pos.npy')
        Tail_H20X_file_path = os.path.join(package_dir, 'resource', 'SimpleFlapper0Tail_H20BLE_X_pos.npy')
        Tail_H20Y_file_path = os.path.join(package_dir, 'resource', 'SimpleFlapper0Tail_H20BLE_Y_pos.npy')
        
        self. LU_bac = BAC(Wing40X_file_path, Wing40Y_file_path, bladeElementNo=40)
        self. LD_bac = BAC(Wing40X_file_path, Wing40Y_file_path, bladeElementNo=40)
        self. RU_bac = BAC(Wing40X_file_path, Wing40Y_file_path, bladeElementNo=40)
        self. RD_bac = BAC(Wing40X_file_path, Wing40Y_file_path, bladeElementNo=40)
        self. rudder_bac = BAC(Tail_V20X_file_path, Tail_V20Y_file_path, bladeElementNo=20)
        self. tail_bac =   BAC(Tail_H20X_file_path, Tail_H20Y_file_path, bladeElementNo=20)
        
        self. FORCE_RELATIVECONSTANT = 0.0005
        self. arena_Air_Density = 1.29
        self. Simulation_Gap    = 1e-3
        self. Controller_Gap_vs_Simulation_Gap = 10
        self. FromWing2Tail = 0.15
        
        
        self. S_d_Flapping_wing_actuator_disk_area = np.pi * 0.15 * 0.15 * 0.5
        
        self. LU_bac. SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
        self. LD_bac. SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
        self. RU_bac. SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
        self. RD_bac. SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
        self. rudder_bac. SetVirturalWingPlaneRelative2Wing([1,0,0,0,-1,0,0,0,-1])
        self. tail_bac.  SetVirturalWingPlaneRelative2Wing([1,0,0,0,1,0,0,0,1])
        
        print("Blade element settings completed!!")
        
        self. Real_motor_LU_RD_joint  = self. flapper.getDevice('LU_RD_Joint')
        self. Real_motor_LD_RU_joint  = self. flapper.getDevice('LD_RU_Joint')

        self. Real_motor_LU_RD_joint_sensor = self. flapper.getDevice('LU_RD_Joint_sensor')
        self. Real_motor_LD_RU_joint_sensor = self. flapper.getDevice('LD_RU_Joint_sensor')

        self. Real_motor_LU_RD_joint_sensor.enable(1)
        self. Real_motor_LD_RU_joint_sensor.enable(1)
        
        self. Real_motor_LU_wing_joint = self. flapper.getDevice('LU_Joint')
        self. Real_motor_LD_wing_joint = self. flapper.getDevice('LD_Joint')
        self. Real_motor_RU_wing_joint = self. flapper.getDevice('RU_Joint')
        self. Real_motor_RD_wing_joint = self. flapper.getDevice('RD_Joint')
        self. Real_motor_rudder_joint  = self. flapper.getDevice('Rudder_Joint')
        self. Real_motor_H_tail_joint  = self. flapper.getDevice('Tail_Joint')
        
        self. Real_motor_LU_wing_joint_sensor = self. flapper.getDevice('LU_Joint_sensor')
        self. Real_motor_LD_wing_joint_sensor = self. flapper.getDevice('LD_Joint_sensor')
        self. Real_motor_RU_wing_joint_sensor = self. flapper.getDevice('RU_Joint_sensor')
        self. Real_motor_RD_wing_joint_sensor = self. flapper.getDevice('RD_Joint_sensor')
        self. Real_motor_rudder_joint_sensor  = self. flapper.getDevice('Rudder_Joint_sensor')
        self. Real_motor_H_tail_joint_sensor  = self. flapper.getDevice('Tail_Joint_sensor')
        
        self. Real_motor_LU_wing_joint_sensor.enable(1)
        self. Real_motor_LD_wing_joint_sensor.enable(1)
        self. Real_motor_RU_wing_joint_sensor.enable(1)
        self. Real_motor_RD_wing_joint_sensor.enable(1)
        self. Real_motor_rudder_joint_sensor. enable(1)
        self. Real_motor_H_tail_joint_sensor. enable(1)
        
        print("Joint settings completed!!")
        
        self. induced_velocity_record = []
        self. induced_velocity_record_list_length = 1000
        
        
        ### link the flappers and joints to the things in the webots
        self.  TheFlapper = self.flapper.getFromDef('ClawedFlapper')
        
        
        
        self. LU_wing = self.flapper.getFromDef('LU')
        self. LD_wing = self.flapper.getFromDef('LD')
        self. RU_wing = self.flapper.getFromDef('RU')
        self. RD_wing = self.flapper.getFromDef('RD')
        self. rudder  = self.flapper.getFromDef('rudder')
        self. tail    = self.flapper.getFromDef('H_Tail')        
        
        self. Flapper_translation      = self.  TheFlapper.getField('translation')
        self. Flapper_rotation         = self.  TheFlapper.getField('rotation')
        
        
        # Set the initial position and orientation of the flapper.
        self.initial_translation = [0, 0, 0]
        self.initial_rotation_in_axis_angle = [1, 0 ,0, 0]
        self.initial_vel = [0, 0, 0, 0, 0, 0]
        
        self. Flapper_translation.setSFVec3f(self.initial_translation)
        self. Flapper_rotation.setSFRotation(self.initial_rotation_in_axis_angle)
        self. TheFlapper. setVelocity(self.initial_vel)
        
        # Initialize the orientation vectors of each part.
        self. update_orientation_vector()
        
        # Initialize the rotation matrices.
        self. update_orientation_mat()
        
        # Initialize the velocity vectors.
        self. update_vel_vector()
        
        self. Flapping_wing_induced_flow_Tail = np.array([0,0,0])
        self. Flapping_wing_induced_flow      = np.array([0,0,0])
        self. vel_FreeFlow                    = np.array([0,0,0])
        self. StrokeAmp    = np.pi / 8
        self. StrokeAmpOffSet = self. StrokeAmp
        
        # Initialize the joint value.
        self. Real_motor_LU_wing_joint_sensor_value = self. Real_motor_LU_wing_joint_sensor.getValue()
        self. Real_motor_LD_wing_joint_sensor_value = self. Real_motor_LD_wing_joint_sensor.getValue()
        self. Real_motor_RU_wing_joint_sensor_value = self. Real_motor_RU_wing_joint_sensor.getValue()
        self. Real_motor_RD_wing_joint_sensor_value = self. Real_motor_RD_wing_joint_sensor.getValue()
        self. Real_motor_rudder_joint_sensor_value  = self. Real_motor_rudder_joint_sensor.getValue()
        self. Real_motor_H_tail_joint_sensor_value  = self. Real_motor_H_tail_joint_sensor.getValue()
        
        print("Motor settings completed!!")
        
        ###------Controller Initialisation-------###
        self. Gamma_now = np.mat([0,0,1]).T
        self. Gamma_last = self. Gamma_now

        self. alt_int = 0.0
        self. alt_int_mag_max = 2.0
        self. alt_err_mag_max = 4.0
        self. alt_vel_mag_max = 3.0

        self. alt_now = 0.0
        self. alt_last = 0.0
        self. alt_des = 0.0


        ###------Parameter Settings------###
        self. roll_rudder_mag_max = 0.7
        self. H_tail_mag_max      = 0.5

        
        Zero_av = np.matrix([[0],[0],[0]])
        self. Angular_velocity_filter = LPSF(Zero_av, 8, 0.8, self. Simulation_Gap)
        
        self. RecordCount = 0
        self. Time_stamp = 0.0
        self. Theta       = 0
        self. StrokeFreq  = 10
        print("Parameters settings completed!!")

    def update_orientation_vector(self):
        self.LU_OrVec = self.LU_wing.getOrientation()
        self.LD_OrVec = self.LD_wing.getOrientation()
        self.RU_OrVec = self.RU_wing.getOrientation()
        self.RD_OrVec = self.RD_wing.getOrientation()
        self.rudder_OrVec = self.rudder.getOrientation()
        self.tail_OrVec   = self.tail.getOrientation()
        self.Flapper_OrVec = self.TheFlapper.getOrientation()
        
    def update_orientation_mat(self):
        self. LU_wing_Rotation_matrix = self. generate_rot_M(self. LU_wing)
        self. LD_wing_Rotation_matrix = self. generate_rot_M(self. LD_wing)
        self. RU_wing_Rotation_matrix = self. generate_rot_M(self. RU_wing)
        self. RD_wing_Rotation_matrix = self. generate_rot_M(self. RD_wing)
        self. rudder_Rotation_matrix = self. generate_rot_M(self. rudder)
        self. tail_Rotation_matrix = self. generate_rot_M(self. tail)
        self. Flapper_Rotation_current = self. generate_rot_M(self. TheFlapper)
        
    def update_vel_vector(self):
        self. LU_velVec = self. LU_wing.getVelocity()
        self. LD_velVec = self. LD_wing.getVelocity()
        self. RU_velVec = self. RU_wing.getVelocity()
        self. RD_velVec = self. RD_wing.getVelocity()
        self. rudder_velVec = self. rudder.getVelocity()
        self. tail_velVec = self. tail.getVelocity()
        self. TheFlapper_velVec = self. TheFlapper.getVelocity()
        
        
        
    def generate_rot_M(self, rigid_body):
        OrVec = rigid_body.getOrientation()
        return np.matrix([[OrVec[0], OrVec[1], OrVec[2]],\
                        [OrVec[3], OrVec[4], OrVec[5]],\
                        [OrVec[6], OrVec[7], OrVec[8]]])
        
    def get_force_in_local(self, locat_matrix, force_in_inertia):
        force_in_inertia_mat = np.mat([force_in_inertia[0],force_in_inertia[1],force_in_inertia[2]]).T
        force_in_local_mat   = locat_matrix.T * force_in_inertia_mat
        force_in_local = np.array([force_in_local_mat[0,0], force_in_local_mat[1,0], force_in_local_mat[2,0]])
        return force_in_local    
        
    def step(self):
        self. Time_stamp = self. Time_stamp  + self. Simulation_Gap
        rclpy.spin_once(self.__node, timeout_sec=0)
        # self.publisher_.publish(self.sphere_def)
        
        # print("Spin start!!")
        self. Theta = self. Theta +  2 * np.pi * self. Simulation_Gap *  self. StrokeFreq
        if (self. Theta > 2 * np.pi):
            self. Theta = np.mod(self. Theta,  2 * np.pi )
        
        LU_RD_joint_angle = self. StrokeAmp * np.sin(self. Theta) + self. StrokeAmpOffSet
        LD_RU_joint_angle = -LU_RD_joint_angle
        
        self. Real_motor_LU_RD_joint.setPosition(LU_RD_joint_angle)
        self. Real_motor_LD_RU_joint.setPosition(LD_RU_joint_angle)
        
        self. update_orientation_mat()
        self. update_orientation_vector()
        
        self. LU_bac.RequestWingOrientation2InertiaFrame (self. LU_OrVec)
        self. LD_bac.RequestWingOrientation2InertiaFrame (self. LD_OrVec)
        self. RU_bac.RequestWingOrientation2InertiaFrame (self. RU_OrVec)
        self. RD_bac.RequestWingOrientation2InertiaFrame (self. RD_OrVec)
        self. rudder_bac.RequestWingOrientation2InertiaFrame (self. rudder_OrVec)
        self. tail_bac.RequestWingOrientation2InertiaFrame (self. tail_OrVec)
        
        self. LU_bac.RequestWingPlaneDirection()
        self. LD_bac.RequestWingPlaneDirection()
        self. RU_bac.RequestWingPlaneDirection()
        self. RD_bac.RequestWingPlaneDirection()
        self. rudder_bac.RequestWingPlaneDirection()
        self. tail_bac.RequestWingPlaneDirection()
        
        self. update_vel_vector()
        
        Flapper_Rotation_current_T = np.transpose(self. Flapper_Rotation_current)
        self. Flapper_Angular_velocity_current = np.matmul(Flapper_Rotation_current_T,\
            np.matrix([[self. TheFlapper_velVec[3]],[self. TheFlapper_velVec[4]],[self. TheFlapper_velVec[5]]]))
        
        
        # Compute the joint sensor value and its derivative.
        # The derivative of the flapping wing cannot be directly obtained by webots sensor,
        # such that we generate it by simple one-step differentiation.
        self. Last_Real_motor_LU_wing_joint_sensor_value = self. Real_motor_LU_wing_joint_sensor_value
        self. Last_Real_motor_LD_wing_joint_sensor_value = self. Real_motor_LD_wing_joint_sensor_value
        self. Last_Real_motor_RU_wing_joint_sensor_value = self. Real_motor_RU_wing_joint_sensor_value
        self. Last_Real_motor_RD_wing_joint_sensor_value = self. Real_motor_RD_wing_joint_sensor_value
        self. Last_Real_motor_rudder_joint_sensor_value = self. Real_motor_rudder_joint_sensor_value
        self. Last_Real_motor_H_tail_joint_sensor_value = self. Real_motor_H_tail_joint_sensor_value
        
        self. Real_motor_LU_wing_joint_sensor_value = self. Real_motor_LU_wing_joint_sensor.getValue()
        self. Real_motor_LD_wing_joint_sensor_value = self. Real_motor_LD_wing_joint_sensor.getValue()
        self. Real_motor_RU_wing_joint_sensor_value = self. Real_motor_RU_wing_joint_sensor.getValue()
        self. Real_motor_RD_wing_joint_sensor_value = self. Real_motor_RD_wing_joint_sensor.getValue()
        self. Real_motor_H_tail_joint_sensor_value  = self. Real_motor_rudder_joint_sensor.getValue()
        
        self. d_Real_motor_LU_wing_joint_sensor_value = (self. Real_motor_LU_wing_joint_sensor_value - self. Last_Real_motor_LU_wing_joint_sensor_value)\
                                                        / self. Simulation_Gap
        self. d_Real_motor_LD_wing_joint_sensor_value = (self. Real_motor_LD_wing_joint_sensor_value - self. Last_Real_motor_LD_wing_joint_sensor_value)\
                                                        /self. Simulation_Gap
        self. d_Real_motor_RU_wing_joint_sensor_value = (self. Real_motor_RU_wing_joint_sensor_value - self. Last_Real_motor_RU_wing_joint_sensor_value)\
                                                        /self. Simulation_Gap
        self. d_Real_motor_RD_wing_joint_sensor_value = (self. Real_motor_RD_wing_joint_sensor_value - self. Last_Real_motor_RD_wing_joint_sensor_value)\
                                                        /self. Simulation_Gap
        self. d_Real_motor_rudder_joint_sensor_value  = (self. Real_motor_rudder_joint_sensor_value - self. Last_Real_motor_rudder_joint_sensor_value)\
                                                        /self. Simulation_Gap
        self. d_Real_motor_H_tail_joint_sensor_value  = (self. Real_motor_H_tail_joint_sensor_value - self. Last_Real_motor_rudder_joint_sensor_value)\
                                                        /self. Simulation_Gap
                                                        
        self. LU_bac.RequestVelocities(self. vel_FreeFlow,self. LU_velVec[0:3],\
                                          self. LU_velVec[3:6],self. d_Real_motor_LU_wing_joint_sensor_value)
        self. LD_bac.RequestVelocities(self. vel_FreeFlow,self. LD_velVec[0:3],\
                                          self. LD_velVec[3:6],self. d_Real_motor_LD_wing_joint_sensor_value)
        self. RU_bac.RequestVelocities(self. vel_FreeFlow,self. RU_velVec[0:3],\
                                          self. RU_velVec[3:6],self. d_Real_motor_RU_wing_joint_sensor_value)
        self. RD_bac.RequestVelocities(self. vel_FreeFlow,self. RD_velVec[0:3],\
                                          self. RD_velVec[3:6],self. d_Real_motor_RD_wing_joint_sensor_value)
        
        # Computing the period length of the flapping wings
        self. Period_length = int( round( 1 / self. StrokeFreq / self. Simulation_Gap ) )
        
        # Record the induced velocity in histroy.
        induced_velocity_record_length = self. induced_velocity_record.__len__()
        induced_velocity_record_list_length_max = 1000
        
        # Update the induced vel list
        if (  self. Period_length < induced_velocity_record_length ):
            ave_of_vel_in_Period_length = \
                sum ( self. induced_velocity_record[ - self. Period_length : induced_velocity_record_length] ) \
                / self. Period_length
            caudal_direction  = [0,0,-1]
            caudal_direction_in_inertia_raw = np. matmul( self. Flapper_Rotation_current, caudal_direction)
            caudal_direction_in_inertia     = np.array(caudal_direction_in_inertia_raw).flatten().tolist()
            mag_in_caudal_ave_vel       = sum (np. multiply(ave_of_vel_in_Period_length, \
                                            caudal_direction_in_inertia ) )
            if (mag_in_caudal_ave_vel < 0):
                mag_in_caudal_ave_vel = 0 
            
            trace_back_length = round( self. FromWing2Tail / (mag_in_caudal_ave_vel + self. FORCE_RELATIVECONSTANT) /   self. Simulation_Gap)
            if (trace_back_length < induced_velocity_record_length):
                self. Flapping_wing_induced_flow_Tail = self. induced_velocity_record[ - trace_back_length]
            else:
                self. Flapping_wing_induced_flow_Tail = 0
        while (induced_velocity_record_list_length_max < self. induced_velocity_record.__len__()):
            self. induced_velocity_record. pop(0)
            
        self. rudder_bac.RequestVelocities(self. vel_FreeFlow +  self. Flapping_wing_induced_flow_Tail, self. rudder_velVec[0:3],\
                                          self. rudder_velVec[3:6],self. d_Real_motor_rudder_joint_sensor_value)
        self. tail_bac.RequestVelocities(self. vel_FreeFlow + self. Flapping_wing_induced_flow_Tail, self. tail_velVec[0:3],\
                                          self. tail_velVec[3:6],0)  

        self. LU_bac.CalcEffectiveVelocity()
        self. LD_bac.CalcEffectiveVelocity()
        self. RU_bac.CalcEffectiveVelocity()
        self. RD_bac.CalcEffectiveVelocity()
        self. rudder_bac.CalcEffectiveVelocity()
        self. tail_bac.CalcEffectiveVelocity()
        
        self. LU_bac.CalcAoA()
        self. LD_bac.CalcAoA()
        self. RU_bac.CalcAoA()
        self. RD_bac.CalcAoA()
        self. rudder_bac.CalcAoA()
        self. tail_bac.CalcAoA()
        
        self. LU_bac.CopmputeAerodynamicForce()
        self. LD_bac.CopmputeAerodynamicForce()
        self. RU_bac.CopmputeAerodynamicForce()
        self. RD_bac.CopmputeAerodynamicForce()
        self. rudder_bac.CopmputeAerodynamicForce()
        self. tail_bac.CopmputeAerodynamicForce()
        
        
        # Compute all the aerodynamic forces.
        self. LU_lift = np.array(np.sum(self. LU_bac. F_t_lift, axis=0)).squeeze()
        self. LD_lift = np.array(np.sum(self. LD_bac. F_t_lift, axis=0)).squeeze()
        self. RU_lift = np.array(np.sum(self. RU_bac. F_t_lift, axis=0)).squeeze()
        self. RD_lift = np.array(np.sum(self. RD_bac. F_t_lift, axis=0)).squeeze()   
        
        # print('LU_lift:', self. LU_lift)
        
        self. LU_drag = np.array(np.sum(self. LU_bac. F_t_drag, axis=0)).squeeze()
        self. LD_drag = np.array(np.sum(self. LD_bac. F_t_drag, axis=0)).squeeze()
        self. RU_drag = np.array(np.sum(self. RU_bac. F_t_drag, axis=0)).squeeze()
        self. RD_drag = np.array(np.sum(self. RD_bac. F_t_drag, axis=0)).squeeze()

        self. LU_r = np.array(np.sum(self. LU_bac. F_r, axis=0)).squeeze()    
        self. LD_r = np.array(np.sum(self. LD_bac. F_r, axis=0)).squeeze()
        self. RU_r = np.array(np.sum(self. RU_bac. F_r, axis=0)).squeeze()
        self. RD_r = np.array(np.sum(self. RD_bac. F_r, axis=0)).squeeze()
        
        self. LU_a = np.array(np.sum(self. LU_bac. F_a, axis=0)).squeeze()    
        self. LD_a = np.array(np.sum(self. LD_bac. F_a, axis=0)).squeeze()
        self. RU_a = np.array(np.sum(self. RU_bac. F_a, axis=0)).squeeze()
        self. RD_a = np.array(np.sum(self. RD_bac. F_a, axis=0)).squeeze()


        self. rudder_lift = np.array(np.sum(self. rudder_bac. F_t_lift, axis=0)).squeeze()
        self. rudder_drag = np.array(np.sum(self. rudder_bac. F_t_drag, axis=0)).squeeze()
        self. rudder_r = np.array(np.sum(self. rudder_bac. F_r, axis=0)).squeeze()
        self. rudder_a = np.array(np.sum(self. rudder_bac. F_a, axis=0)).squeeze()


        self. tail_lift = np.array(np.sum(self. tail_bac. F_t_lift, axis=0)).squeeze()
        self. tail_drag = np.array(np.sum(self. tail_bac. F_t_drag, axis=0)).squeeze()
        self. tail_r = np.array(np.sum(self. tail_bac. F_r, axis=0)).squeeze()
        self. tail_a = np.array(np.sum(self. tail_bac. F_a, axis=0)).squeeze()
        
        relative = True
        
        # Add all the drag force in the wing inertia frame.
        LU_drag_in_wing = self. get_force_in_local(self. LU_wing_Rotation_matrix, self. LU_drag)
        self. LU_wing. addForceWithOffset([LU_drag_in_wing[0],LU_drag_in_wing[1],LU_drag_in_wing[2]],\
            [self. LU_bac.X_pos_t,self. LU_bac.Y_pos_t,0],relative)
        LD_drag_in_wing = self. get_force_in_local(self. LD_wing_Rotation_matrix, self. LD_drag)
        self. LD_wing. addForceWithOffset([LD_drag_in_wing[0],LD_drag_in_wing[1],LD_drag_in_wing[2]],\
            [self. LD_bac.X_pos_t,self. LD_bac.Y_pos_t,0],relative)
        RU_drag_in_wing = self. get_force_in_local(self. RU_wing_Rotation_matrix, self. RU_drag)
        self. RU_wing. addForceWithOffset([RU_drag_in_wing[0],RU_drag_in_wing[1],RU_drag_in_wing[2]],\
            [self. RU_bac.X_pos_t,self. RU_bac.Y_pos_t,0],relative)
        RD_drag_in_wing = self. get_force_in_local(self. RD_wing_Rotation_matrix, self. RD_drag)
        self. RD_wing. addForceWithOffset([RD_drag_in_wing[0],RD_drag_in_wing[1],RD_drag_in_wing[2]],\
            [self. RD_bac.X_pos_t,self. RD_bac.Y_pos_t,0],relative)
        rudder_drag_in_wing = self. get_force_in_local(self. rudder_Rotation_matrix, self. rudder_drag)
        self. rudder. addForceWithOffset([rudder_drag_in_wing[0],rudder_drag_in_wing[1],rudder_drag_in_wing[2]],\
            [self. tail_bac. X_pos_t, self. rudder_bac. Y_pos_t, 0],relative)
        tail_drag_in_wing = self. get_force_in_local(self. tail_Rotation_matrix, self. tail_drag)
        self. tail. addForceWithOffset([tail_drag_in_wing[0],tail_drag_in_wing[1],tail_drag_in_wing[2]],\
            [self. tail_bac. X_pos_t, self. tail_bac. Y_pos_t, 0],relative)
        
        # print('self.RecordCount:',self.RecordCount)
        if self.RecordCount > 0:
            LU_lift_in_wing = self. get_force_in_local(self. LU_wing_Rotation_matrix, self. LU_lift)
            self. LU_wing.addForceWithOffset([LU_lift_in_wing[0],LU_lift_in_wing[1],LU_lift_in_wing[2]],\
                [self. LU_bac.X_pos_t,self. LU_bac.Y_pos_t,0],relative)
            LD_lift_in_wing = self. get_force_in_local(self. LD_wing_Rotation_matrix, self. LD_lift)
            self. LD_wing.addForceWithOffset([LD_lift_in_wing[0],LD_lift_in_wing[1],LD_lift_in_wing[2]],\
                [self. LD_bac.X_pos_t,self. LD_bac.Y_pos_t,0],relative)
            RU_lift_in_wing = self. get_force_in_local(self. RU_wing_Rotation_matrix, self. RU_lift)
            self. RU_wing.addForceWithOffset([RU_lift_in_wing[0],RU_lift_in_wing[1],RU_lift_in_wing[2]],\
                [self. RU_bac.X_pos_t,self. RU_bac.Y_pos_t,0],relative)
            RD_lift_in_wing = self. get_force_in_local(self. RD_wing_Rotation_matrix, self. RD_lift)
            self. RD_wing.addForceWithOffset([RD_lift_in_wing[0],RD_lift_in_wing[1],RD_lift_in_wing[2]],\
                [self. RD_bac.X_pos_t,self. RD_bac.Y_pos_t,0],relative)
            rudder_lift_wing = self. get_force_in_local(self. rudder_Rotation_matrix, self. rudder_lift)
            self. rudder .addForceWithOffset([rudder_lift_wing[0],rudder_lift_wing[1],rudder_lift_wing[2]],\
                [0, self. rudder_bac. Y_pos_t, 0],relative)
            tail_lift_wing = self. get_force_in_local(self. tail_Rotation_matrix, self. tail_lift)
            self. tail   .addForceWithOffset([tail_lift_wing[0],tail_lift_wing[1],tail_lift_wing[2]] ,\
                [self. tail_bac. X_pos_t, self. tail_bac. Y_pos_t, 0],relative)
        
        
        Flapper_Rotation_current_T = np.transpose(self. Flapper_Rotation_current)
        LU_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, self. LU_lift)).squeeze()
        LD_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, self. LD_lift)).squeeze()
        RU_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, self. RU_lift)).squeeze()
        RD_lift_in_real = np.array(np.matmul(Flapper_Rotation_current_T, self. RD_lift)).squeeze()
        Total_lift_in_real =  (LU_lift_in_real + LD_lift_in_real + RU_lift_in_real + RD_lift_in_real)
        Flapping_wing_induced_flow_raw_list = - 0.5 * GetUnitDirection_Safe(Total_lift_in_real) * \
                                np.sqrt( 0.5 * np.linalg.norm(Total_lift_in_real) /\
                                         self. S_d_Flapping_wing_actuator_disk_area / self. arena_Air_Density )
    
        Flapping_wing_induced_flow_raw_mat  = np.matrix([[Flapping_wing_induced_flow_raw_list[0]],\
                                                        [Flapping_wing_induced_flow_raw_list[1]],\
                                                        [Flapping_wing_induced_flow_raw_list[2]]])
        Flapping_wing_induced_flow = 2 * np.array(np.matmul(self. Flapper_Rotation_current, Flapping_wing_induced_flow_raw_mat)).squeeze()
        
        # Capture the airflow created by the flapping wings, which will be utilized when it flows towards the tail later on.
        self. induced_velocity_record. append( Flapping_wing_induced_flow )
        
          
        LU_r_in_wing = self. get_force_in_local(self. LU_wing_Rotation_matrix, self. LU_r)
        self. LU_wing.addForceWithOffset([LU_r_in_wing[0],LU_r_in_wing[1],LU_r_in_wing[2]],[self. LU_bac.X_pos_r,self. LU_bac.Y_pos_r,0],relative)
        LD_r_in_wing = self. get_force_in_local(self. LD_wing_Rotation_matrix, self. LD_r)
        self. LD_wing.addForceWithOffset([LD_r_in_wing[0],LD_r_in_wing[1],LD_r_in_wing[2]],[self. LD_bac.X_pos_r,self. LD_bac.Y_pos_r,0],relative)
        RU_r_in_wing = self. get_force_in_local(self. RU_wing_Rotation_matrix, self. RU_r)
        self. RU_wing.addForceWithOffset([RU_r_in_wing[0],RU_r_in_wing[1],RU_r_in_wing[2]],[self. RU_bac.X_pos_r,self. RU_bac.Y_pos_r,0],relative)
        RD_r_in_wing = self. get_force_in_local(self. RD_wing_Rotation_matrix, self. RD_r)
        self. RD_wing.addForceWithOffset([RD_r_in_wing[0],RD_r_in_wing[1],RD_r_in_wing[2]],[self. RD_bac.X_pos_r,self. RD_bac.Y_pos_r,0],relative)
    
        
    
        LU_a_in_wing = self. get_force_in_local(self. LU_wing_Rotation_matrix, self. LU_a)
        self. LU_wing.addForceWithOffset([LU_a_in_wing[0],LU_a_in_wing[1],LU_a_in_wing[2]],[self. LU_bac.X_pos_a,self. LU_bac.Y_pos_a,0],relative)
        LD_a_in_wing = self. get_force_in_local(self. LD_wing_Rotation_matrix, self. LD_a)
        self. LD_wing.addForceWithOffset([LD_a_in_wing[0],LD_a_in_wing[1],LD_a_in_wing[2]],[self. LD_bac.X_pos_a,self. LD_bac.Y_pos_a,0],relative)
        RU_a_in_wing = self. get_force_in_local(self. RU_wing_Rotation_matrix, self. RU_a)
        self. RU_wing.addForceWithOffset([RU_a_in_wing[0],RU_a_in_wing[1],RU_a_in_wing[2]],[self. RU_bac.X_pos_a,self. RU_bac.Y_pos_a,0],relative)
        RD_a_in_wing = self. get_force_in_local(self. RD_wing_Rotation_matrix, self. RD_a)
        self. RD_wing.addForceWithOffset([RD_a_in_wing[0],RD_a_in_wing[1],RD_a_in_wing[2]],[self. RD_bac.X_pos_a,self. RD_bac.Y_pos_a,0],relative)


        self. torsion_spring_constant = 0.025
        self. Angular_velocity_filter. march_forward(self. Flapper_Angular_velocity_current)
        
        self. Gamma_now = Flapper_Rotation_current_T * np.mat( [0,0,1] ).T
        self. Flapper_translation_value = self. Flapper_translation.getSFVec3f()
        
        
        if ( np.mod(self. RecordCount, self. Controller_Gap_vs_Simulation_Gap)==0):
        # These values relate to the FWAV mounting. 
            # print('Control started!')
            
            Gamma_des_x = -0.5 #Gamma_des_x_list[Gamma_des_x_i]
            Gamma_des_y = 0 #Gamma_des_y_list[Gamma_des_y_i]
            Gamma_des_z = np.sqrt(1 - Gamma_des_x**2 - Gamma_des_y**2)

            Gamma_now_x = self. Gamma_now[0,0]
            Gamma_now_y = self. Gamma_now[1,0]
            Gamma_now_z = self. Gamma_now[2,0]
            
            Gamma_des = np.mat( [Gamma_des_x, Gamma_des_y, Gamma_des_z] ).T
            Gamma_des = Gamma_des/ np.linalg.norm(Gamma_des)
            omega_now = self. Angular_velocity_filter.Get_filtered()

            omega_x = omega_now[0,0]
            omega_y = omega_now[1,0]
            omega_z = omega_now[2,0]
            
            self. anuglar_vel = AngularVel()
            self. anuglar_vel. time_stamp = self. Time_stamp 
            self. anuglar_vel. x = omega_x
            self. anuglar_vel. y = omega_y
            self. anuglar_vel. z = omega_z
            
            self. angular_vel_publisher.publish(self. anuglar_vel)
            
            k_rud = 1
            k_ele = 1
            k_omega_x = 5e-2
            k_omega_y = 5e-2

            roll_rudder_amplitude = - k_rud * (Gamma_des_y * Gamma_now_z - Gamma_des_z * Gamma_now_y) + k_omega_x * omega_x
            H_tail_amplitude = k_ele * (Gamma_des_z * Gamma_now_x - Gamma_des_x * Gamma_now_z) - k_omega_y * omega_y

            self. alt_now = self. Flapper_translation_value[2]
            alt_vel_des = 0 #alt_des_vel_list[alt_des_vel_i]
            # alt_vel_des = -0.3
            
            self. alt_des = self. alt_des + alt_vel_des * (self. Controller_Gap_vs_Simulation_Gap * self.Simulation_Gap)
            
            alt_vel = (self. alt_now - self. alt_last) / (self. Controller_Gap_vs_Simulation_Gap * self.Simulation_Gap)
            
            self. alt_last = self. alt_now
            
            k_i = 0.1
            k_p = 10
            k_d = 3
            
            e_alt = k_p * (self. alt_des - self. alt_now)
            e_alt_vel = k_d * (alt_vel_des - alt_vel)
            
            e_alt = max (- self. alt_err_mag_max, min(self. alt_err_mag_max, e_alt))
            # e_alt_vel = max (- alt_vel_mag_max, min(alt_vel_mag_max, e_alt_vel))
        
            
            self. alt_int = self. alt_int + k_i * (self. alt_des - self. alt_now)
            self. alt_int = max (- self. alt_int_mag_max, min(self. alt_int_mag_max, self. alt_int))
            
            
            
            StrokeFreq_def =  11
            self. StrokeFreq = StrokeFreq_def + self. alt_int + e_alt + e_alt_vel
                    

            
            
            self. roll_rudder_amplitude = max( - self. roll_rudder_mag_max, \
                                    min ( roll_rudder_amplitude, self. roll_rudder_mag_max))
            
            self. H_tail_amplitude = max ( - self. H_tail_mag_max, \
                                    min (H_tail_amplitude, self. H_tail_mag_max))
            
        self. Real_motor_H_tail_joint. setPosition(self. H_tail_amplitude)
        
        self. Real_motor_rudder_joint. setPosition(self. roll_rudder_amplitude)
        
        self. Real_motor_LU_wing_joint.setTorque(- self. torsion_spring_constant \
                                        * self. Real_motor_LU_wing_joint_sensor_value )
        self. Real_motor_LD_wing_joint.setTorque(- self. torsion_spring_constant \
                                        * self. Real_motor_LD_wing_joint_sensor_value )
        self. Real_motor_RU_wing_joint.setTorque(- self. torsion_spring_constant \
                                        * self. Real_motor_RU_wing_joint_sensor_value )
        self. Real_motor_RD_wing_joint.setTorque(- self. torsion_spring_constant \
                                        * self. Real_motor_RD_wing_joint_sensor_value )
        
       
        
        self. RecordCount = self. RecordCount + 1

        # print("Spin!!")
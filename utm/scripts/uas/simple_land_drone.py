#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
To do

"""

from asynchat import simple_producer
import os
from re import L
import rospy
import airsim
import random
from geometry_msgs.msg import PoseStamped
from utm import Database
import numpy as np
import math as m
import re


class PID():
    """Compensator"""
    def __init__(self, kp, ki, kd, rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pre_error = 0.0    
        self.pre_ierror = 0.0
        self.rate = rate

    def compute_p_error(self, desired, actual):
        """get errors"""
        return desired - actual

    def __compute_i_error(self, error):
        """compute i gain with trapezoid rule"""
        return self.pre_ierror + ((error + self.pre_error)) * (1/self.rate)

    def __compute_d__error(self, error):
        """compute d gain"""
        return (error - self.pre_error)/(1/self.rate)

    def compute_gains(self, desired, actual):
        """return PID gains"""
        error = self.compute_p_error(desired, actual)
        i_error = self.__compute_i_error(error)
        d_error = self.__compute_d__error(error)

        gain = (self.kp*error)+ (self.ki*i_error) + (self.kd*d_error)

        self.pre_error = error
        self.pre_ierror = i_error

        return gain,error


class SimpleLandDrone():
    """Control the SimpleFlight AirsimDrone"""
    def __init__(self, vehicle_name, wsl_ip, api_port):
        self.client = airsim.MultirotorClient(ip=str(wsl_ip), port=api_port)
        self.world_client =  airsim.VehicleClient(ip=str(wsl_ip), port=api_port)
        self.vehicle_name = vehicle_name
        
        self.global_pos_sub = rospy.Subscriber("/"+vehicle_name+"/global_position/pose", 
                                        PoseStamped,self.global_pos_cb)    
        
        self.offset_x = rospy.get_param("~offset_x", 0)
        self.offset_y = rospy.get_param("~offset_y", -10)

        self.pid = PID(kp=0.75,ki=0.0,kd=0.0,rate=20)

        self.init_vel = rospy.get_param("~init_vel", 4)

        self.global_enu_pos = [None,None,None]
        self.global_enu_quat = [None,None,None, None]

        self.target_pos = [None,None,None, None]

        self.path_planning_service = Database.PathPlannerService()

        #this is bad need to take in the bubble as a parm
        self.col_bubble = 1
        self.col_radius = self.col_bubble/2
        self.bubble_bounds = list(np.arange(-self.col_radius, self.col_radius+1, 1))
        
        #this is also bad need to figure out how to parameterize what kind of bubble to use
        string_name = (re.findall('\d+', self.vehicle_name))
        #get second number string values after PX4_
        self.waypoint_num = string_name[1]

        #need to check the uav sub thing
        self.track_sub = rospy.Subscriber("uav0/kf_tag/pose", PoseStamped, self.track_cb)
            
        self.get_start_position()

    def init_drone(self):
        """initalize drone and send query to plan a path"""
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle_name)
        #val = self.client.armDisarm(False, self.vehicle_name)
        #self.takeoff_drone()
        #self.send_enu_waypoint(self.start_position, self.init_vel)

    def get_start_position(self):
        """get starting position from params"""
        x = rospy.get_param("~init_x",  5)
        y = rospy.get_param("~init_y", 5)
        z = rospy.get_param("~init_z", 15)

        self.start_position = [x,y,z]

    def takeoff_drone(self):
        """commands drone to takeoff"""
        self.client.takeoffAsync(vehicle_name=self.vehicle_name)

    def global_pos_cb(self, msg):
        """returns global ENU position of drone from subscribing to global 
        position topic"""
        enu_x = msg.pose.position.x
        enu_y = msg.pose.position.y
        enu_z = msg.pose.position.z

        enu_qx = msg.pose.orientation.x 
        enu_qy = msg.pose.orientation.y
        enu_qz = msg.pose.orientation.z
        enu_qw = msg.pose.orientation.w

        self.global_enu_pos = [enu_x, enu_y, enu_z]
        self.global_enu_quat = [enu_qx, enu_qy, enu_qz, enu_qw]

    def track_cb(self,msg):
        """callback for info of drone"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        self.target_pos = [x, y, z]

    def convert_enu_to_ned(self, enu_coords):
        """converts enu position to ned""" 
        ned_x = enu_coords[1]
        ned_y = enu_coords[0]
        ned_z = -enu_coords[2]
        #print("ned_coords",ned_x,ned_y,ned_z)
        return [ned_x, ned_y, ned_z]
        
    def arm_disarm(self, arm_disarm):
        """true to arm, false to disarm"""
        self.client.armDisarm(arm_disarm, self.vehicle_name)
        # arm_disarm = True
        # while simple_drone.arm_disarm(arm_disarm) != True:
        #     simple_drone.arm_disarm(arm_disarm) 
            
        #     if simple_drone.arm_disarm(arm_disarm) == True:
        #         print("disarmed")
        
    def land_veh(self):
        """land vehicle"""
        self.client.landAsync(vehicle_name=self.vehicle_name)
        
    def compute_offsets(self,enu_wp, tag=False):
        """send global commands have to subtract the offsets"""
        enu_global_x = enu_wp[0] - self.offset_x
        if tag == True:
            enu_global_y = enu_wp[1] #+ self.offset_y
        else:
            enu_global_y = enu_wp[1] - self.offset_y
        enu_global_z = enu_wp[2] #- 0.8 #add these height offset because it can be weird
        return [enu_global_x, enu_global_y, enu_global_z]
 
    def send_enu_waypoint(self, enu_wp, velocity):
        """send enu waypoint command to drone by converting to ned to use api
        takes in the enu waypoint and the desired velocity"""
        
        ## spawn waypoints before computing offsets
        enu_wp = self.compute_offsets(enu_wp)
        ned_wp = self.convert_enu_to_ned(enu_wp)
        #join tells it to wait for task to complete
        #self.moveToPosition(ned_wp,velocity)
        self.client.moveToPositionAsync(ned_wp[0], ned_wp[1],
         ned_wp[2], velocity, vehicle_name=self.vehicle_name).join()

    def moveToPosition(self, desired_ned, v):
        """currentPos is in unreal engine coordinate from left hand convention"""
        currentPos = self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated.position        
        
        error = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 \
            + (currentPos.z_val - desired_ned[2])**2)**0.5
        
        t = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 \
            + (currentPos.z_val - desired_ned[2])**2)**0.5 / v
        
        tol = 0.5
        time_tol = 0.15
        while abs(error)>=tol:
            print("error is", error)
            if error <=tol or t <= time_tol: 
                print("im done")
                self.client.moveByVelocityAsync(0, 0, 0 , t, vehicle_name=self.vehicle_name)
                self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
                rospy.sleep(t)
                return
            t = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 \
                + (currentPos.z_val - desired_ned[2])**2)**0.5 / v
            
            x_gain,x_err = self.pid.compute_gains(desired_ned[0], currentPos.x_val)
            y_gain,y_err = self.pid.compute_gains(desired_ned[1], currentPos.y_val)
            z_gain,z_err = self.pid.compute_gains(desired_ned[2], currentPos.y_val)

            # delta_x = desired_ned[0] - currentPos.x_val
            # delta_y = desired_ned[1] - currentPos.y_val
            delta_z = desired_ned[2] - currentPos.z_val                
            vx = x_gain/t
            vy = y_gain/t
            vz = delta_z/t
                
            self.client.moveByVelocityAsync(vx, vy, vz, t, vehicle_name=self.vehicle_name).join()
            rospy.sleep(t)
            currentPos = self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated.position        

            #rospy.sleep(t)
            error = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 + (currentPos.z_val - desired_ned[2])**2)**0.5


    def prec_land(self, velocity):
        """track target and do precision landing on target"""
        # while abs(self.global_enu_pos[2]) >= 0.5:
        des_pos = self.compute_offsets(self.target_pos, True)
        print(des_pos)
        ned_wp = self.convert_enu_to_ned(des_pos)
        # #join tells it to wait for task to complete
        self.client.moveToPositionAsync(ned_wp[0], ned_wp[1],
            -15, velocity, vehicle_name=self.vehicle_name)
        
if __name__ == '__main__':
    rospy.init_node("simple_land", anonymous=False)
    #should take in a ros param for veh_name
    veh_name = rospy.get_param("~veh_name", 'PX4_0')
    wsl_ip = os.getenv('WSL_HOST_IP')
    api_port = rospy.get_param("~api_port", 41451)
    
    simple_drone = SimpleLandDrone(veh_name, wsl_ip, api_port)
    simple_drone.init_drone()
    simple_drone.takeoff_drone()
    simple_drone.send_enu_waypoint(simple_drone.start_position, simple_drone.init_vel)
    
    rate_val = 20
    rate = rospy.Rate(rate_val)
    
    while not rospy.is_shutdown():
        # simple_drone.send_enu_waypoint(simple_drone.start_position, simple_drone.init_vel)

        simple_drone.prec_land(0.5)
    rate.sleep()
    

    

    
    
    
    
    
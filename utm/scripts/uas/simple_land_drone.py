#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
To do

"""

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
        
        self.offset_x = rospy.get_param("~offset_x", -10)
        self.offset_y = rospy.get_param("~offset_y", 20)

        self.pid = PID(kp=0.75,ki=0.0,kd=0.0,rate=20)

        self.init_vel = rospy.get_param("~init_vel", 4)

        self.global_enu_pos = [None,None,None]
        self.global_enu_quat = [None,None,None, None]

        self.path_planning_service = Database.PathPlannerService()

        #this is bad need to take in the bubble as a parm
        self.col_bubble = 1
        self.col_radius = self.col_bubble/2
        self.bubble_bounds = list(np.arange(-self.col_radius, self.col_radius+1, 1))
        
        #this is also bad need to figure out how to parameterize what kind of bubble to use
        string_name = (re.findall('\d+', self.vehicle_name))
        #get second number string values after PX4_
        self.waypoint_num = string_name[1]

    def init_drone(self):
        """initalize drone and send query to plan a path"""
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle_name)
        # val = self.client.armDisarm(False, self.vehicle_name)
        #self.takeoff_drone()
        #self.send_enu_waypoint(self.start_position, self.init_vel)

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
        
    def arm_disarm(self, arm_disarm):
        """true to arm, false to disarm"""
        self.client.armDisarm(arm_disarm, self.vehicle_name)
        
    def land_veh(self):
        """land vehicle"""
        self.client.landAsync(vehicle_name=self.vehicle_name)
        
if __name__ == '__main__':
    rospy.init_node("simple_land", anonymous=False)
    #should take in a ros param for veh_name
    veh_name = rospy.get_param("~veh_name", 'PX4_0')
    wsl_ip = os.getenv('WSL_HOST_IP')
    api_port = rospy.get_param("~api_port", 41451)
    
    simple_drone = SimpleLandDrone(veh_name, wsl_ip, api_port)
    simple_drone.init_drone()
    
    #simple_drone.land_veh()
    arm_disarm = True
    while simple_drone.arm_disarm(arm_disarm) != True:
        simple_drone.arm_disarm(arm_disarm) 
        
        if simple_drone.arm_disarm(arm_disarm) == True:
            print("disarmed")
    

    
    
    
    
    
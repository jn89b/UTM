#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from ipaddress import ip_address
import pandas as pd
#from itertools import itercycle

import rospy
import sys
import airsim 
import tempfile
import os
import numpy as np
import cv2
import pprint

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

"""
What is the best way to do this? I want to make a better wrapper for Airsim
- read csv 
- append object name and set as the nodespace 
- publish the global positions of the UAVS as ENU
- publish the UAVs transformation
- publish the UAVS global gps coordinates

The big problem is I don't want to publish this as a for loop I want to do this
at same time relatively basically multi thread or multiprocess, lets try this first
and evaluate the scalibility of this method
"""

"""c drive is equalivant to /mnt/c/"""
FILEPATH = "/mnt/c/Users/jnguy/Documents/AirSim/"
FILENAME = "airsim_test.csv"

class AirsimROSWrapper():
    """
    takes in information of uav
    instantiates airsimdroneros for n uavs and appends to list
    then publishes the wrapped messages
    """

    def __init__(self, wsl_ip):
        """Houses the API calls"""
        self.client = airsim.MultirotorClient(ip=str(wsl_ip))

    def get_global_uav_location(self, uav_name):
        """get orientation of UAV"""
        return self.client.simGetObjectPose("PX4_"+str(uav_name)) 

    def main(self, n_uavs):
        """this is the main loop through and publish the messages"""
        for i in range(0,n_uavs):
            airsim_drone = AirsimDroneROS(name ="PX4_"+str(i))
            ned_pose = self.get_global_uav_location(i)
            enu_position, enu_quat = self.convert_ned_to_enu(ned_pose)
            airsim_drone.publish_global_position(enu_position, enu_quat)

    def convert_ned_to_enu(self, ned_pose):
        """convert ned convention of airsim to ned for ROS publishing"""
        enu_x = ned_pose.position.y_val
        enu_y = ned_pose.position.x_val
        enu_z = -ned_pose.position.z_val
        enu_quat_x =  ned_pose.orientation.y_val
        enu_quat_y =  ned_pose.orientation.x_val
        enu_quat_z =  -ned_pose.orientation.x_val
        enu_quat_w =  ned_pose.orientation.w_val

        enu_position = [enu_x, enu_y, enu_z]
        enu_quat = [enu_quat_x, enu_quat_y, enu_quat_z, enu_quat_w]

        return enu_position, enu_quat

class AirsimDroneROS():
    """
    _airsim_client = Airsims API wrapper 
    _name = uav name from csv
    """
    def __init__(self, name):
        self._name = name 
        self._global_pos_pub = self.generate_global_pub()

    def generate_global_pub(self):
        """generate global position publshier of drone"""
        topic_name = self._name +"/global_position/pose"
        
        return rospy.Publisher(topic_name, PoseStamped, queue_size = 10)

    def publish_global_position(self, position_vector, orientation_vector):
        """publish global position of UAV wrt ROS coordinate frame of ENU"""        
        now = rospy.Time.now()
        position_msg = PoseStamped()
        position_msg.header.frame_id = str(self._name) +"_wrap"
        position_msg.header.stamp = now
        position_msg.pose.position.x = position_vector[0]
        position_msg.pose.position.y = position_vector[1]
        position_msg.pose.position.z = position_vector[2]
        position_msg.pose.orientation.x = orientation_vector[0]
        position_msg.pose.orientation.y = orientation_vector[1]
        position_msg.pose.orientation.z = orientation_vector[2]
        position_msg.pose.orientation.w = orientation_vector[3]
        print(self._name)
        print("published message ", position_msg)
        self._global_pos_pub.publish(position_msg)

def get_landing_zone_locs(n_zones):
    for i in range(1,n_zones):
        print("Position of zone", client.simGetObjectPose("AprilTag_Character_"+str(i)))

def get_global_uav_location(n_uavs, airsim_client):
    """get orientation of UAV"""
    client = airsim_client.client
    for i in range(0,n_uavs):
        val = client.simGetObjectPose("PX4_"+str(i))
        print("Global Position of UAV"+str(i), val.position) 

def get_local_uavs(n_uavs):
    for i in range(0,n_uavs):
        print("Local Position of UAV"+str(i), client.simGetVehiclePose("PX4_"+str(i)))

def create_ros_wrapper(dataframe):
    """ros wrapper to define the global location of the uavs"""
    uav_list = []
    for index, row in dataframe.iterrows():
        pass
    
if __name__ == '__main__':    
    rospy.init_node("test_airsim_wrap", anonymous=True)
    wsl_ip = os.getenv('WSL_HOST_IP')
    print("my address is", wsl_ip)
    df = pd.read_csv(FILEPATH+FILENAME)
    create_ros_wrapper(df)

    airsim_ros = AirsimROSWrapper(wsl_ip)
    airsim_ros.main(n_uavs=3)
    #get_global_uav_location(n_uavs=4, airsim_client = airsim_client)




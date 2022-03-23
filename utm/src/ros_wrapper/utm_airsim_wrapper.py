#!/usr/bin/env python
# -*- coding: utf-8 -*-
from utm import config

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
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

"""
What is the best way to do this? I want to make a better wrapper for Airsim
- read csv 
- append object name and set as the nodespace 
- publish the global positions of the UAVS as ENU -X
- publish the uav transformations based on odometry - X
- publish the UAVS global gps coordinates

The big problem is I don't want to publish this as a for loop I want to do this
at same time relatively basically multi thread or multiprocess, lets try this first
and evaluate the scalibility of this method

PUBLISH THE 
"""

"""c drive is equalivant to /mnt/c/"""
FILEPATH = config.FILEPATH
FILENAME = config.FILENAME

class AirsimROSWrapper():
    """
    takes in information of uav
    instantiates airsimdroneros for n uavs and appends to list
    then publishes the wrapped messages
    """

    def __init__(self, wsl_ip):
        """Houses the API calls"""
        self.client = airsim.MultirotorClient(ip=str(wsl_ip))

    def get_object_global_location(self, object_name):
        """get orientation of UAV"""
        return self.client.simGetObjectPose(object_name) 

    def convert_ned_to_enu(self, ned_pose):
        """convert ned convention of airsim to ned for ROS publishing"""
        enu_x = ned_pose.position.y_val
        enu_y = ned_pose.position.x_val
        enu_z = -ned_pose.position.z_val 
        enu_quat_x =  ned_pose.orientation.y_val
        enu_quat_y =  ned_pose.orientation.x_val
        enu_quat_z =  -ned_pose.orientation.z_val
        enu_quat_w =  ned_pose.orientation.w_val

        enu_position = [enu_x, enu_y, enu_z]
        enu_quat = [enu_quat_x, enu_quat_y, enu_quat_z, enu_quat_w]

        return enu_position, self.__normalize_quat(enu_quat)

    def __normalize_quat(self, quat_list):
        """normalize the quaternion"""
        quat_vector = np.array(quat_list)
        magnitude =  np.linalg.norm(quat_vector)
        
        return quat_vector/magnitude
    
    def generate_global_pub(self,topic_name):
        """generate global position publshier of drone -> private method"""
        topic_name = topic_name
        
        return rospy.Publisher(topic_name, PoseStamped, queue_size = 10)

    def broadcast_global_transform(self, frame_id, position_vector,orientation_vector):
        """broadcast global transformation of drone"""
        self._br = tf.TransformBroadcaster()
        self._br.sendTransform(
            (position_vector[0], position_vector[0], position_vector[2]),
            (orientation_vector[0], orientation_vector[1], orientation_vector[2],
            orientation_vector[3]),
            rospy.Time.now(),
            str(frame_id)+"_wrap",
            "world_enu")

    def publish_global_position(self, publisher, frame_id_name, position_vector, orientation_vector):
        """publish global position of UAV wrt ROS coordinate frame of ENU"""        
        now = rospy.Time.now()
        position_msg = PoseStamped()
        position_msg.header.frame_id = frame_id_name
        position_msg.header.stamp = now
        position_msg.pose.position.x = position_vector[0]
        position_msg.pose.position.y = position_vector[1]
        #added 0.5 here because there is a weird offset with Unreal and RVIZ position
        position_msg.pose.position.z = position_vector[2] + 0.5 
        position_msg.pose.orientation.x = orientation_vector[0]
        position_msg.pose.orientation.y = orientation_vector[1]
        position_msg.pose.orientation.z = orientation_vector[2]
        position_msg.pose.orientation.w = orientation_vector[3]
        publisher.publish(position_msg)
        self.broadcast_global_transform(frame_id_name, position_vector, orientation_vector)

class AirsimObjectROS():
    """
    _airsim_client = Airsims API wrapper 
    _name = uav name from csv
    """
    def __init__(self, name):
        self._name = name 
        self._global_pos_pub = self.__generate_global_pub()
        self._br = tf.TransformBroadcaster()

    def __generate_global_pub(self):
        """generate global position publshier of drone -> private method"""
        topic_name = self._name +"/global_position/pose"
        
        return rospy.Publisher(topic_name, PoseStamped, queue_size = 10)

    def __broadcast_global_transform(self,time, position_vector,orientation_vector):
        """broadcast global transformation of drone"""
        self._br.sendTransform(
            (position_vector[0], position_vector[0], position_vector[2]),
            (orientation_vector[0], orientation_vector[1], orientation_vector[2],
            orientation_vector[3]),
            rospy.Time.now(),
            str(self._name)+"_wrap",
            "world_enu")

    def publish_global_position(self, position_vector, orientation_vector):
        """publish global position of UAV wrt ROS coordinate frame of ENU"""        
        now = rospy.Time.now()
        position_msg = PoseStamped()
        position_msg.header.frame_id = str(self._name) +"_wrap"
        position_msg.header.stamp = now
        position_msg.pose.position.x = position_vector[0]
        position_msg.pose.position.y = position_vector[1]
        #added 0.5 here because there is a weird offset with Unreal and RVIZ position
        position_msg.pose.position.z = position_vector[2] + 0.5 
        position_msg.pose.orientation.x = orientation_vector[0]
        position_msg.pose.orientation.y = orientation_vector[1]
        position_msg.pose.orientation.z = orientation_vector[2]
        position_msg.pose.orientation.w = orientation_vector[3]
        self._global_pos_pub.publish(position_msg)
        self.__broadcast_global_transform(now, position_vector, orientation_vector)
#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from utm import config

from ipaddress import ip_address
import pandas as pd

import os
import roslaunch
import roslaunch.remote
import time, sys, subprocess
import rospy
import std_msgs.msg
import datetime
import rospkg

"""
to do:
    -read csv files
    -get the wsl ip address
    -launch airsim node:
        - set the host ip as the wsl ip address
        - for n drones remap the camera topic:
        <arg name="uav_0_camera_name" default="/airsim_node/PX4_0/downwards_custom_0"/>   
        <arg name="uav_0_camera_info" default="$(arg uav_0_camera_name)/camera_info"/>
        <remap from="/airsim_node/PX4_0/downwards_custom_0/Scene/camera_info" to="$(arg uav_0_camera_info)"/>

    = run n mavros nodes

    - run my custom ros wrapper node:
        -set the directory of the CSV file? 
"""
class RosLaunchFile():
    def __init__(self, pkg_name, launch_file_name):
        self.pkg_name = pkg_name
        self.launch_file_name = launch_file_name
    
    def set_arg_list(self, arg_list):
        """set up any arguments for launch file"""
        self.client_args = [self.pkg_name, self.launch_file_name, arg_list]

    def start_launch(self):
        """begin initiating of launch file"""
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        return roslaunch.rlutil.resolve_launch_arguments(self.client_args)

class ROSNode():
    def __init__(self, ros_pkg, node_name, node_type):
        self._ros_pkg = ros_pkg
        self._node_name = node_name
        self._node_type = node_type

    def set_args(self, args):
        """define args has to be a string only format"""
        self._args = args
        print("argument is ", self._args)

    def get_node(self):
        """return node format"""
        self.node = roslaunch.core.Node(
            package=self._ros_pkg,
            node_type=self._node_type,
            name=self._node_name,
            args=str(self._args),
            output='screen')
        print(self.node)
        return self.node

def get_uav_names(dataframe):
    """return list of uav names from dataframe"""

    return df['uav_name'].to_list()

def get_airsim_launch(package_name):
    """I have a code duplicate"""
    launch_file_name = "airsim_node.launch"    
    host_arg = 'host:='+str(wsl_ip)
    airsim_launch_args = [package_name, launch_file_name, host_arg]
    airsim_launch_file = roslaunch.rlutil.resolve_launch_arguments(airsim_launch_args)
    
    return airsim_launch_file, airsim_launch_args

def get_mavros_launch(package_name):
    """I have a code duplicate"""
    launch_file_name = "mavros_bridge.launch"    
    host_arg = 'host:='+str(wsl_ip)
    mavros_launch_args = [package_name, launch_file_name, host_arg]
    mavros_launch_file = roslaunch.rlutil.resolve_launch_arguments(mavros_launch_args)
    
    return mavros_launch_file, mavros_launch_args
    
if __name__ == "__main__":
    rospy.init_node('utm_sim_launch', anonymous=True)
    wsl_ip = os.getenv('WSL_HOST_IP')
    df = pd.read_csv(config.FILEPATH+config.FILENAME)
    uav_name_list = get_uav_names(df)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    host_arg = 'host:='+str(wsl_ip)

    #airsim launch file
    airsim_pkg = 'airsim_ros_pkgs' 
    airsim_launch = 'airsim_node.launch'
    airsim_args = [airsim_pkg , airsim_launch]
    #mavros
    utm_pkg_name = 'utm'
    mavros_launch = 'mavros_bridge.launch'
    mavros_args = [utm_pkg_name, mavros_launch]

    #pass this if i have no arguments to make
    roslaunch_file1 = (roslaunch.rlutil.resolve_launch_arguments(airsim_args))[0]
    roslaunch_file2 = (roslaunch.rlutil.resolve_launch_arguments(mavros_args))[0]

    launch_files = [roslaunch_file1, roslaunch_file2]

    controlportlocal = 14540
    controlportexit = 14557
    tgt_system = 1

    uav_bridge_list = []
    for idx, uav_name in enumerate(uav_name_list):
        print(idx)
        client_args = ['utm', 'mavros_communication.launch', 
        'namespace:=uav'+str(idx), 'ControlPortLocal:='+str(controlportlocal+idx),
        'ControlPortLocal:='+str(controlportlocal+idx), 
        'tgt_system:='+str(tgt_system+idx)]
        mavros_bridge_file = (roslaunch.rlutil.resolve_launch_arguments(client_args)[0], client_args[2:])        

        launch_files.append(mavros_bridge_file)

    print("launch files are", launch_files)
    launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()

    try:
        launch.spin()
    finally:
        launch.shutdown()
        

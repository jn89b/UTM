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

import subprocess

"""
to do:
    -read csv files
    -get the wsl ip address
    -launch airsim node: -DONE 
        - set the host ip as the wsl ip address
        - for n drones remap the camera topic:
        <arg name="uav_0_camera_name" default="/airsim_node/PX4_0/downwards_custom_0"/>   
        <arg name="uav_0_camera_info" default="$(arg uav_0_camera_name)/camera_info"/>
        <remap from="/airsim_node/PX4_0/downwards_custom_0/Scene/camera_info" to="$(arg uav_0_camera_info)"/>

    = run n mavros nodes - DONE
    - run my custom ros wrapper node -DONE:
        -set the directory of the CSV file?
    - run n offboard nodes for uavs 
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
    
if __name__ == "__main__":
    rospy.init_node('utm_sim_launch', anonymous=False)
    
    wsl_ip = os.getenv('WSL_HOST_IP')
    df = pd.read_csv(config.FILEPATH+config.FILENAME)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    uav_name_list = get_uav_names(df)
    launch = roslaunch.scriptapi.ROSLaunch()

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

    ## MAVROS COMMUNICATION BRIDGE
    controlportlocal = 14540
    controlportexit = 14557
    tgt_system = 1
    for idx, uav_name in enumerate(uav_name_list):
        client_args = ['utm', 'mavros_communication.launch', 
        'namespace:=uav'+str(idx), 'ControlPortLocal:='+str(controlportlocal+idx),
        'ControlPortLocal:='+str(controlportlocal+idx), 
        'tgt_system:='+str(tgt_system+idx)]
        mavros_bridge_file = (roslaunch.rlutil.resolve_launch_arguments(client_args)[0], client_args[2:])        
        launch_files.append(mavros_bridge_file)

    ## UAS OPERATOR
    for idx, uav in df.iterrows():
        x_spawn = uav['spawn_x']
        y_spawn = uav['spawn_y']
        z_spawn = uav['spawn_z']
        init_x = uav['init_x']
        init_y = uav['init_y']
        init_z = uav['init_z']

        client_args = ['utm', 'uas_operator.launch', 
                    'namespace:=uav'+str(idx), 'offset_x:='+str(y_spawn),
                    'offset_y:='+str(x_spawn), 'init_x:='+str(init_x),
                    'init_y:='+str(init_y), 'init_z:='+str(init_z)]
        uas_launch = (roslaunch.rlutil.resolve_launch_arguments(client_args)[0], client_args[2:])        
        launch_files.append(uas_launch)
    
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()

    ## RUN MY ROSWRAPPER
    node = roslaunch.core.Node('utm', 'test_airsim_api.py')
    launch.launch(node)

    try:
        launch.spin()
    finally:
        launch.shutdown()
        

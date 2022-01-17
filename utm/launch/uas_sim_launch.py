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
- run tag detection
    - remap topic name
    - insert camera name argument
- run apriltag transformation
    - tag_id
    - tag_wrt_uav
    - tag_frame_id
    
"""

def get_uav_names(dataframe):
    """return list of uav names from dataframe"""

    return df['uav_name'].to_list()

if __name__ == "__main__":
    rospy.init_node('uas_sim_launch', anonymous=True)
    wsl_ip = os.getenv('WSL_HOST_IP')
    df = pd.read_csv(config.FILEPATH+config.FILENAME)
    uav_name_list = get_uav_names(df)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    uav_name_list = get_uav_names(df)
    launch = roslaunch.scriptapi.ROSLaunch()

    launch_files = []
    ## TRANSFORMATIONS FOR FIDUCIAL TAGS
    """
    transformations of the tags are currently set to uav_name+_wrap
    """
    for idx, uav in df.iterrows():
        uav_name = uav['uav_name']
        client_args = ['utm', 'tag_transform.launch', 
                    'namespace:=uav'+str(idx), 
                    'camera_name:=/airsim_node/'+str(uav_name)+'/downwards_custom_'+ str(idx),
                    'quad_tf:='+str(uav_name)+'_wrap',
                    'rtag_drone:=/tag_wrt_'+str(uav_name)]
        uas_launch = (roslaunch.rlutil.resolve_launch_arguments(client_args)[0], client_args[2:])        
        launch_files.append(uas_launch)

    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    
    try:
        launch.spin()
    finally:
        launch.shutdown()
        
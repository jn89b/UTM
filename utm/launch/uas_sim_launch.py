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
    ## UAS OPERATOR
    for idx, uav in df.iterrows():
        x_spawn = uav['spawn_x']
        #y_spawn = uav['spawn_y']
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
        # node = roslaunch.core.Node('utm', 'offboard_test', 
        #          namespace='/uav'+str(idx), args='offboard_test/offset_x:='+str(y_spawn))


    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    launch.start()
    time.sleep(2)
    
    #launch.launch.start(node)
    ## BEGIN launching filles
    #launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
    #launch.launch(uas_launch_file)

    ## terminal command for PX4
    #p = subprocess.Popen([command, argument1,...], cwd=working_directory)
    #subprocess.check_call(['your_command', 'arg 1', 'arg 2'], cwd=working_dir)
    #p = subprocess.check_output(['sh','./Tools/sitl_multiple_run.sh '], cwd=config.PX4PATH)

    try:
        launch.spin()
    finally:
        launch.shutdown()
        
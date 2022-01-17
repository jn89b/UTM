#!/usr/bin/env python
# -*- coding: utf-8 -*- 

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

class ROSLaunchFile():
    def __init__(self, pkg_name, launch_file_name):
        self.pkg_path = self.__get_ros_pkg_path(pkg_name)
        self.launch_file_name = launch_file_name
        self.full_path = [str(pkg_path)+"/launch/"+launch_file_name]

    def __get_ros_pkg_path(self, package_name):
        #https://wiki.ros.org/Packages#Client_Library_Support
        """get ros package"""
        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # list all packages, equivalent to rospack list
        rospack.list() 
        # get the file path for rospy_tutorials
        return rospack.get_path(package_name)

def get_ros_pkg_path(package_name):
    #https://wiki.ros.org/Packages#Client_Library_Support
    """get ros package"""
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    # list all packages, equivalent to rospack list
    rospack.list() 
    # get the file path for rospy_tutorials
    return rospack.get_path(package_name)    

if __name__ == "__main__":
    rospy.init_node('en_Mapping', anonymous=True)

    pkg_path = get_ros_pkg_path('utm')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

    utm_pkg_name = 'utm'
    cli_args1 = [utm_pkg_name, 'multiple_drone.launch']
    cli_args2 = [utm_pkg_name, 'multiple_drone_mission.launch']

    roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
    roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)

    launch_files = [roslaunch_file1, roslaunch_file2]

    launch = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file1)
    launch.start()

    """this is where you make sure the launch file keeps runnings"""
    try:
        launch.spin()
    finally:
    # After Ctrl+C, stop all nodes from running
        launch.shutdown()

    """
    launch.start()
    rospy.loginfo("started")
    rospy.sleep(30)
    # 3 seconds later
    launch.shutdown()
    """


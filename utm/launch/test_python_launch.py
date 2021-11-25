#!/usr/bin/env python
# -*- coding: utf-8 -*- 
# http://wiki.ros.org/roslaunch/API%20Usage

"""
To do:
launch file 
https://github.com/Shilpaj1994/ROS-Pylauncher
"""


import roslaunch
import roslaunch.remote
import time, sys, subprocess
from sets import Set
import rospy
import std_msgs.msg
import datetime

NODES_STILL_RUNNING = Set()

LAST_TIME_NODES_STILL_RUNNING_CHANGED = datetime.datetime.now()

def start_nodes():
    pass

"""


DEFINE DRONE PARAMS
for n drones remap cameras:
    camera_name : /airsim_node/PX4_0/downwards_custom_0
    camera_info : camera_name/camera_info
    udp and gcp port numbers
    tgt_system number
    quad_tf 
    world_tf
    camera_name
    uav_name
    
    offsets are defined by json file

LAUNCH AIRSIM 
Set up 1 param for :
mavlink_bridge_url: WSL2_IP 
host_ip: WSL2_IP

Launch Airsim ros file
Run mavros bridge
add remaps here for camera_info

Set up group namespace for n_drone_name:
    launch the following files:
        mavros_gcs_launch 
        broadcast_tf_launch
        offboard_launch
    nodes:
        uav_mong_query
        moving_avg_main

https://answers.ros.org/question/352529/how-do-you-set-namespace-in-a-python-program/
https://medium.com/@shilpajbhalerao/ros-pylauncher-9ac50951e230
"""

state = False

if __name__ == "__main__":
    rospy.init_node('en_Mapping', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/justin/catkin_ws/src/utm/utm/launch/multiple_drone.launch"])
    roslaunch.configure_logging(uuid)
    launch_path_name = ["/home/justin/catkin_ws/src/utm/utm/launch/multiple_drone.launch"]
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, launch_path_name)
    launch.start()

    # Start another node
    #node = roslaunch.core.Node(utm)
    #launch.launch(node)

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
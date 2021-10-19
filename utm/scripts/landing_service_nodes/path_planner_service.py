#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
from datetime import *

from utm import Database

if __name__ == '__main__':
    rospy.init_node('utm_path_planner')
    pathPlanner = Database.ZonePlanner()
    uavs,zone_names = pathPlanner.find_assigned_zones(0)
    zone_coord_list = pathPlanner.get_zone_wp_list(zone_names)
    uav_class_list = pathPlanner.generate_publishers(uavs)
    """generate publishers and begin sending waypoint commands to drones""" 
    threads = []
    #open multiple threads to begin publising the waypoint command to the drone

    rate_val = 10
    rate = rospy.Rate(rate_val)

    """wrap this guy in main function"""
    while not rospy.is_shutdown():
        """need to check when the class is empty, if empty we listen for more drones"""
        #for i in range(len(uav_class_list)):
        for idx, uav in enumerate(uav_class_list[:]):
            uav.send_utm_state_command(0)
            uav.send_waypoint_command(zone_coord_list[idx])

        if uav_class_list is None:
            uavs,zone_names = pathPlanner.find_assigned_zones()
            zone_coord_list = pathPlanner.get_zone_wp_list(zone_names)
            uav_class_list = pathPlanner.generate_publishers(uavs)
        rate.sleep()





#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import threading
from utm import Database

from datetime import *


if __name__ == '__main__':
    rospy.init_node('landing_state_service')
    landingStateService = Database.ZonePlanner()
    uavs,zone_names = landingStateService.find_assigned_zones(0)
    zone_coord_list = landingStateService.get_zone_wp_list(zone_names)
    uav_class_list = landingStateService.generate_publishers(uavs)
    print(uav_class_list)

    """generate publishers and begin sending waypoint commands to drones""" 
    threads = []
    print("Hello")
    #open multiple threads to begin publising the waypoint command to the drone

    rate_val = 10
    rate = rospy.Rate(rate_val)

    """wrap this guy in main function"""
    while not rospy.is_shutdown():
        for idx, uav in enumerate(uav_class_list[:]):
            uav.send_utm_state_command(2)
            print(uav.name)
        rate.sleep()



    

    


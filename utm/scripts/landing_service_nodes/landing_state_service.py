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
            #landingStateService.update_uav_state(uav.name)

            #print(uav_class_list[i].uav_pos_pub)
            #uav_class_list[i].send_waypoint_command(zone_coord_list[i])
            #t = threading.Thread(target=uav.send_utm_state_command(2), args=(idx,))
            #t.start()
            #threads.append(t)
        #rospy.spin()
        rate.sleep()



    

    


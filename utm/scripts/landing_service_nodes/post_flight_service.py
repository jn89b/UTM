#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
from datetime import *

from utm import Database

class PostFlight():
    """
    Listen for UAV battery
    Check if uav is in state 2 
    If state 2 check if it has a postflight plan
    If not plan a postflight trajectory
    Send drone off with postfligth trajectory
    """

    def __init__(self):
        pass

    def plan_uav_path(self):
        """request information about uav home location and 
        send it trajectory path"""

    def begin_postflight(self):
        """main implementation"""

if __name__ == '__main__':
    rospy.init_node('postflight_service')
    landingStateService = Database.ZonePlanner()
    uavs,zone_names = landingStateService.find_assigned_zones(0)
    zone_coord_list = landingStateService.get_zone_wp_list(zone_names)
    uav_class_list = landingStateService.generate_publishers(uavs)
    print(uav_class_list)

    rate_val = 10
    rate = rospy.Rate(rate_val)

    while not rospy.is_shutdown():
        for idx, uav in enumerate(uav_class_list[:]):
            uav.send_utm_state_command(3)
            uav.send_waypoint_command([0,0,0])

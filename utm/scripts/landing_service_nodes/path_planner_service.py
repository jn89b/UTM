#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
from datetime import *

from utm import Database

class PathPlannerService():
    """
    listen for uavs that have a waypoint based on state_service 0
    get coordinates and send waypoints
    check if drone has arrived at the waypoint or not
    if drone has reached final waypoint based on distance then update state_service
    """
    previous_service_number = 0
    update_service_number = 1

    def __init__(self):
        self.pathPlanner = Database.ZonePlanner()
    

    def main(self):
        uavs,zone_names = self.pathPlanner.find_assigned_zones(self.previous_service_number)
        zone_coord_list = self.pathPlanner.get_zone_wp_list(zone_names)
        uav_class_list = self.pathPlanner.generate_publishers(uavs)

        rate_val = 1
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            """need to check when the class is empty, if empty we listen for more drones"""
            #for i in range(len(uav_class_list)):
            if not uav_class_list:
                print("Waiting for uavs")
                rospy.sleep(5)
                uavs,zone_names = self.pathPlanner.find_assigned_zones(self.previous_service_number)
                zone_coord_list = self.pathPlanner.get_zone_wp_list(zone_names)
                uav_class_list = self.pathPlanner.generate_publishers(uavs)
            else:
                for idx, uav in enumerate(uav_class_list[:]):
                    print("sending waypoint command")
                    rospy.sleep(1) #wait for a couple of seconds
                    uav.send_utm_state_command(self.previous_service_number)
                    uav.send_waypoint_command(zone_coord_list[idx])
                                        
                    if self.pathPlanner.is_arrived_to_zone(zone_coord_list[idx], uav.coords):
                        self.pathPlanner.update_uav_state(uav.name,self.update_service_number)
                        uav_class_list.remove(uav)
                        print(uav.name + " has reached the waypoint")
                    
                    if not uav_class_list:
                        break

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('utm_path_planner')
    pathPlannerService = PathPlannerService()
    pathPlannerService.main()






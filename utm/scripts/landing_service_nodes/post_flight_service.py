#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
from datetime import *

from utm import Database

class PostFlightService():
    """
    Listen for UAV battery
    Check if uav is in state 2 
    If state 2 check if it has a postflight plan
    If not plan a postflight trajectory
    Send drone off with postfligth trajectory
    check if drone has left area if so, update the landing zone as vacant 
    and remove uav
    """
    previous_service_number = 2
    update_service_number = 3

    def __init__(self):
        self.postFlight = Database.ZonePlanner()

    def plan_uav_path(self):
        """request information about uav home location and 
        send it trajectory path"""

    def main(self):
        uavs,zone_names = self.postFlight.find_assigned_zones(self.previous_service_number)
        print(uavs)
        zone_coord_list = self.postFlight.get_zone_wp_list(zone_names)
        uav_class_list = self.postFlight.generate_publishers(uavs)
        
        rate_val = 10
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():

            if not uav_class_list:
                print("Waiting for uavs")
                rospy.sleep(5)
                uavs,zone_names = self.postFlight.find_assigned_zones(self.previous_service_number)
                zone_coord_list = self.postFlight.get_zone_wp_list(zone_names)
                uav_class_list = self.postFlight.generate_publishers(uavs)
            else:
                for idx, uav in enumerate(uav_class_list[:]):
                    print("sending post command")
                    rospy.sleep(0.5) #wait for a couple of seconds
                    #print(uav.coords)
                    uav.send_utm_state_command(self.update_service_number)
                    uav.send_waypoint_command([0,0,0])

                    if self.postFlight.has_left_zone(zone_coord_list[idx], uav.coords):
                        self.postFlight.update_uav_state(uav.name,self.update_service_number)
                        uav_class_list.remove(uav)
                        print(uav.name + " has left the area")
 
                    if not uav_class_list:
                        break


            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('post_flight_service')
    postFlightService = PostFlightService()
    postFlightService.main()
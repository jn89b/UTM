#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
import threading
from bson.objectid import ObjectId

import rospy
from datetime import *

from utm import Database
import multiprocessing 
from threading import Thread

class PathHomeSenderService():
    """
    Should rename as PathSender

    listen for uavs that have a waypoint based on state_service 0
    get coordinates and send waypoints
    check if drone has arrived at the waypoint or not
    if drone has reached final waypoint based on distance then update state_service
    """
    previous_service_number = 2
    update_service_number = 3

    def __init__(self):
        self.zonePlanner = Database.ZonePlanner()

    def find_uavs_who_have_homepath(self):
        """find uavs that have a service status of 0 but do not have
        a waypoint"""
        uavs = []
        myquery = {"$and": [{"landing_service_status":self.previous_service_number}, 
                    {"Home Waypoints": {'$exists': True}}]}
        cursor = self.zonePlanner.landing_service_col.find(myquery)
        for document in cursor:
            uavs.append(document["uav_name"])

        return uavs

    def send_wp_commands(self,uav_class_list, uav):
        uav.send_utm_state_command(self.update_service_number) # want to arm this guy
        if not uav.coords:
            uav.get_uav_coords()

        """need to open multiple threads and send waypoint commands for drone"""
        waypoint_list = self.zonePlanner.find_uav_home_waypoints(uav.name)
        zone_name = self.zonePlanner.find_uav_zone_name(uav.name)    
        
        for wp in waypoint_list:
            #self.zonePlanner.update_uav_state(uav.name,self.update_service_number)
            print("index", uav.wp_index)
            if uav.wp_index > (len(waypoint_list)-1):
                self.zonePlanner.update_uav_state(uav.name,self.update_service_number)
                uav_class_list.remove(uav)
                self.zonePlanner.update_landing_zone(zone_name)
                print(uav.name + " has reached the final waypoint")
                break
            
            waypoint = waypoint_list[uav.wp_index]
            """check if uav has left zone area"""
            # if self.zonePlanner.has_left_zone(self.zonePlanner.find_zone_waypoints(zone_name), uav.coords) \
            # and trigger == False:
            #     print("setting trigger to true")
            #     trigger = True
            #     self.zonePlanner.update_landing_zone(zone_name)
                
            """badly worded this is if we are at some assigned waypoint"""
            if self.zonePlanner.is_arrived_to_zone(waypoint, uav.coords) == False:
                uav.send_waypoint_command(waypoint)
            else:
                #print("going to next wp")
                uav.wp_index +=1
            
    def main(self):
        """initialize uavs"""
        uavs = self.find_uavs_who_have_homepath()
        uav_class_list = self.zonePlanner.generate_publishers(uavs)

        rate_val = 5
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            """need to check when the class is empty, if empty we listen for more drones"""
            #for i in range(len(uav_class_list)):
            if not uav_class_list: #if nothing then we continue to listen for uavs
                print("Waiting for uavs")
                rospy.sleep(2.0)
                uavs = self.find_uavs_who_have_homepath()
                print("uavs are", uavs)
                uav_class_list = self.zonePlanner.generate_publishers(uavs)
            else:
                threads = []
                for idx, uav in enumerate(uav_class_list[:]):
                    print(uav_class_list)
                    rospy.sleep(1.0) #wait for a couple of seconds
                    t = multiprocessing.Process(self.send_wp_commands(uav_class_list, uav))
                    t.start()
                    threads.append(t)

                    if not uav_class_list:
                        break

                for t in threads:
                    t.join()
                    
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('utm_home_sender')
    pathHomeSenderService = PathHomeSenderService()
    pathHomeSenderService.main()


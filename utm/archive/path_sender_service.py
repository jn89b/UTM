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

class PathSenderService():
    """
    Should rename as PathSender

    listen for uavs that have a waypoint based on state_service 0
    get coordinates and send waypoints
    check if drone has arrived at the waypoint or not
    if drone has reached final waypoint based on distance then update state_service
    """
    previous_service_number = 0
    update_service_number = 1

    def __init__(self):
        self.zonePlanner = Database.ZonePlanner()

    def send_wp_commands(self,uav_class_list, uav):
        uav.send_utm_state_command(self.previous_service_number)
        print(uav.coords)
        """need to open multiple threads and send waypoint commands for drone"""
        waypoint_list = self.zonePlanner.find_uav_waypoints(uav.name)
        
        print("waypoint list", waypoint_list)
        print("len", len(waypoint_list))
        for wp in waypoint_list:
            """check if arrived to final waypoint"""

            if self.zonePlanner.is_arrived_to_zone(waypoint_list[-1], uav.coords, 1.0) == True:
                self.zonePlanner.update_uav_state(uav.name,self.update_service_number)
                uav_class_list.remove(uav)
                print(uav.name + " has reached the final waypoint")
                break        
                
            if uav.wp_index > len(waypoint_list):
                self.zonePlanner.update_uav_state(uav.name,self.update_service_number)
                uav_class_list.remove(uav)
                print(uav.name + " has reached the final waypoint")
                break    
            waypoint = waypoint_list[uav.wp_index]            
            """badly worded this is if we are at some assigned waypoint"""
            if self.zonePlanner.is_arrived_to_zone(waypoint, uav.coords, 4.0) == False:
                uav.send_waypoint_command(waypoint)
                print("sending waypoint command: ", waypoint[0], waypoint[1])
            else:
                print("going to next wp")
                uav.wp_index +=1
                if uav.wp_index >= len(waypoint_list):
                    self.zonePlanner.update_uav_state(uav.name,self.update_service_number)
                    uav.send_waypoint_command(waypoint_list[-1])
                    uav_class_list.remove(uav)
                    print(uav.name + " has reached the final waypoint")
                    break

            
    def check_valid_uav(self,uav):
        if uav.coords != [None,None]:
            return True

    def main(self):
        """initialize uavs need to see if I can generate a list
        more than one so if list length is 1 continue to listen for another or 2
        iterations and append"""

        rate_val = 5
        rate = rospy.Rate(rate_val)
        uavs,zone_names = self.zonePlanner.find_assigned_zones(self.previous_service_number)
        
        if len(uavs) == 1:
            print("seeing if we have more uavs")
            rospy.sleep(5.0)
            uavs,zone_names = self.zonePlanner.find_assigned_zones(self.previous_service_number)
            
        uav_class_list = self.zonePlanner.generate_publishers(uavs)

        while not rospy.is_shutdown():
            """need to check when the class is empty, if empty we listen for more drones"""
            if not uav_class_list: #if nothing then we continue to listen for uavs
                print("no uavs")
                uavs,zone_names = self.zonePlanner.find_assigned_zones(self.previous_service_number)
                if len(uavs) == 1:
                    print("seeing if we have more uavs")
                    rospy.sleep(5.0)
                    uavs,zone_names = self.zonePlanner.find_assigned_zones(self.previous_service_number)

                uav_class_list = self.zonePlanner.generate_publishers(uavs)
            else:
                threads = []
                for idx, uav in enumerate(uav_class_list[:]):
                    if self.check_valid_uav(uav):
                        rospy.sleep(2.0) #wait for a couple of seconds
                        t = multiprocessing.Process(self.send_wp_commands(uav_class_list, uav))
                        t.start()
                        threads.append(t)

                        if not uav_class_list:
                            break
                    else:
                        continue

                for t in threads:
                    t.join()
                    
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('utm_path_planner')
    pathPlannerService = PathSenderService()
    pathPlannerService.main()

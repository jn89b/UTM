#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import threading
from utm import Database

from datetime import *

class LandingStateService():
    
    """
    LandingStateService listens for uavs that are at state 1
    if so check if they are at the correct location 
    and allow permission to land

    needs to keep listening 
    check uav at location
    allow uav to land by setting uav command to 2
    once landed and disarmed we set to state 3
 
    """
    previous_service_number = 1
    update_service_number = 2

    def __init__(self):
        self.landingPlanner = Database.ZonePlanner()

    def main(self):
        uavs,zone_names = self.landingPlanner.find_assigned_zones(self.previous_service_number)
        
        rate_val = 5
        rate = rospy.Rate(rate_val)

        if len(uavs) == 1:
            print("seeing if we have more uavs")
            rospy.sleep(5.0)
            uavs,zone_names = self.landingPlanner.find_assigned_zones(self.previous_service_number)
        
        #zone_coord_list = self.landingPlanner.get_zone_wp_list(zone_names)
        uav_class_list = self.landingPlanner.generate_publishers(uavs)
        
        while not rospy.is_shutdown():
            if not uav_class_list:
                uavs,zone_names = self.landingPlanner.find_assigned_zones(self.previous_service_number)
                if len(uavs) == 1:
                    print("seeing if we have more uavs")
                    rospy.sleep(5.0)
                    uavs,zone_names = self.landingPlanner.find_assigned_zones(self.previous_service_number)
            
                uav_class_list = self.landingPlanner.generate_publishers(uavs)
            else:
                for idx, uav in enumerate(uav_class_list[:]):
                    print("Controling ", uav.name)
                    #rospy.sleep(1) #wait for a couple of seconds
                    uav.send_utm_state_command(self.update_service_number)
                    #print("sending landing")
                    if uav.mode == "AUTO.LAND" and uav.armed == False:
                        self.landingPlanner.update_uav_state(uav.name, self.update_service_number)
                        uav_class_list.remove(uav)
                        #print(uav.name + " has landed")

                    if not uav_class_list:
                        break
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('landing_state_service')
    landingStateService = LandingStateService()
    landingStateService.main()




    

    


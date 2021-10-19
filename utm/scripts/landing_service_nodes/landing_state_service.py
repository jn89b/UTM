#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import threading
from utm import Database, UAVGen
from scipy import spatial

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from mongodb_store_msgs.msg import StringPairList
from mongodb_store.message_store import MessageStoreProxy
from datetime import *

class LandingStateService():
    """
    Looks for Drones who are in State 1 and are close to the location assigned,
    if close we allow the UAV to begin to land 
    once drones have landed we set the state to 2 
    """
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "message_store"
    landing_srv_col_name = "landing_service_db"
    landing_zone_col_name = "landing_zones"

    def __init__(self):
        #access database
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        #recieve collection information
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
        self.landing_zone_col = self.mainDB[self.landing_zone_col_name]

    def find_assigned_zones(self, service_num):
        """check if uav has an assigned location and if they are at state 1
        refactor this code landing state service and path planner are basically
        the same except we are looking for landing_service_status val"""
        #myquery = {"Zone Assignment": {"$exists": True}}
        uavs = []
        zones = []
        myquery = {"$and": [{"landing_service_status": service_num}, 
                    {"Zone Assignment": {'$exists': True}}]}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uavs.append(document["uav_name"])
            zones.append(document["Zone Assignment"])

        return uavs,zones

    def find_zone_waypoints(self, zone_number):
        myquery = {"Zone Number": zone_number}
        cursor = self.landing_zone_col.find(myquery)
        for document in cursor:
            zone_coordinates = document["location"]

        return zone_coordinates

    def get_zone_wp_list(self, zone_names):
        zone_coords = []
        for zone in zone_names:
            zone_coords.append(self.find_zone_waypoints(zone))
        
        return zone_coords

    def generate_publishers(self, uavs):
        uavObject_list = []
        for uav in uavs:
            uavComms = UAVGen.UAVComms(uav)
            uavObject_list.append(uavComms)
        
        return uavObject_list

    def update_uav_state(self, uav_name, new_status):
        self.landing_service_col.update({"uav_name": uav_name},
        {"$set":{
            "landing_service_status": new_status
        }})

if __name__ == '__main__':
    rospy.init_node('landing_state_service')
    landingStateService = LandingStateService()
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



    

    


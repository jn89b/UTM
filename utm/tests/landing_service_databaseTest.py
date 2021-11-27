#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import platform
import math

from utm import Database
from mongodb_store_msgs.msg import StringPairList
from mongodb_store.message_store import MessageStoreProxy

from datetime import *

if float(platform.python_version()[0:2]) >= 3.0:
    import io
else:
    import StringIO

HOME_X = -30
HOME_Y = -40

class PreLandingService():
    """
    Pre Landing Service:
    Assigns Landing Zones with waypoints from 
    Should probably rename as Pre Flight Planner
    """
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "message_store"
    main_col_name = "data_service"
    landing_srv_col_name = "landing_service_db"
    landing_zone_col_name = "landing_zones"
    geofencing_col = None #need to figure out how to set up geofencing in the area
    
    def __init__(self):

        #access database
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        #collections 
        self.main_collection = self.mainDB[self.main_col_name]
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
        self.landing_zone_col = self.mainDB[self.landing_zone_col_name]

        #ros service proxies with mongodb
        self.data_srv_col_prox = MessageStoreProxy(collection=self.main_col_name)

        self.landing_srv_col_prox = MessageStoreProxy(collection=self.landing_srv_col_name)
        self.landing_zone_col_prox = MessageStoreProxy(collection=self.landing_zone_col_name)
        self.zonePlanner = Database.ZonePlanner()

    def find_uavs_needing_wps(self):
        """find uavs that have a service status of 0 but do not have
        a waypoint"""
        uavs = []
        myquery = {"$and": [{"landing_service_status":0}, 
                    {"Raw Waypoint": {'$exists': False}}]}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uavs.append(document["uav_name"])

        return uavs

    def get_uav_info(self, field_name):
        """return field name info where landing service status is at 0
        field_name must be type str"""

        myquery = {"landing_service_status": 0}
        uav_info_list = []
        cursor = self.landing_service_col.find(myquery)
        for document in cursor: 
            uav_info_list.append(document[field_name])

        return uav_info_list

    def compute_2d_euclidean(self, position, goal_position):
        """compute euclidiean with position and goal as 2d vector component"""
        distance =  math.sqrt(((position[0] - goal_position[0]) ** 2) + 
                        ((position[1] - goal_position[1]) ** 2))
        
        return distance

    def prioritize_uavs(self, uav_ids):
        """return sorted dictionary of uavs closest to homebase"""
        priority_dict = dict.fromkeys(uav_ids)
        for key in priority_dict:
            home_position = self.zonePlanner.find_uav_homeposition(key)
            priority_dict[key] = self.compute_2d_euclidean(home_position, [HOME_X, HOME_Y])

        priority_dict = sorted(priority_dict.items(), key=lambda x: x[1], reverse=False) 
        print(priority_dict)

        
if __name__ == '__main__':
    """
    listen for uavs that are requesting a service 
    if uav is not part of the landing service database and is requesting the 
    service then we will 
    add it to the landing service database
    generate message strings with ros? 

    make a list of drone names in database 
    search for names not in this database and is requesting service
    add onto database 
    keep looping
    """

    rospy.init_node("landing_service_database")

    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    
    preLandingService  = PreLandingService()

    try:
        #landingDBNode.listen_for_incoming()
        uav_names = preLandingService.find_uavs_needing_wps()
        sorted_dict = preLandingService.prioritize_uavs(uav_names)
        #uav_home_list = preLandingService.get_uav_info("uav_home")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


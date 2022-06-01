#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
import pymongo
from utm import Database
import numpy as np
from scipy import spatial

from datetime import *
import math
import rospy

"""
landing zones: 
zone 1 = 70,70
zone 2 = 70,85 
zone 3 = 85,85
zone 4 = 85,70
"""

def compute_dist(curr, goal):
    """computes the distance between the current positon and goal inputs should be
    a list """
    p1 = np.array(curr)
    p2 = np.array(goal)

    squared_dist = np.sum((p1-p2)**2, axis=0)
    dist = np.sqrt(squared_dist)

    return dist

class LandingPlanningService():
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "landingServiceDatabase"
    main_col_name = "data_service"
    landing_srv_col_name = "incoming_uas"
    landing_zone_col_name = "landing_zones"
    geofencing_col = None #need to figure out how to set up geofencing in the area
    
    def __init__(self) -> None:
        #access database
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        self.main_collection = self.mainDB[self.main_col_name]
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
    
        #ros service proxies with mongodb
        self.zonePlanner = Database.ZonePlanner()
        self.path_planning_service = Database.PathPlannerService()

    def find_inbound_uavs(self):
        """requests query for open landing zones"""
        myquery = {"uav_state": "inbound"}
        uavs = []
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uav = [document["_id"], document["goal_point"]]
            uavs.append(uav)
            
        return uavs
    
    
if __name__=='__main__':
    pass
    
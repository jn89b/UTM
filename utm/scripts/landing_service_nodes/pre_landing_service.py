#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
from utm import Database
from scipy import spatial

from mongodb_store_msgs.msg import StringPairList
from mongodb_store.message_store import MessageStoreProxy
from datetime import *
"""
To do:
get information of all open availible landing zones in the grida
get information of drones who are flying in and requesting service
get information of geofencing area in grid locations

plan flight for uav based on cost heuristic of minimal distance 
and no collisions

guide UAV to landing zone and set the open landing zone to closed 
set UAV state to 1 

listen for incoming UAVs and check if landing not avail

"""

class PreLandingService():
    """
    PreLandingService
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

    def check_open_zones(self):
        myquery = {"Vacant": True}
        cursor = self.landing_zone_col.find(myquery)
        if cursor.count() == 0:
            return False

    def find_open_zones(self):
        """requests query for open landing zones"""
        myquery = {"Vacant": True}
        open_zone_names = []
        open_zone_coordinates= []
        cursor = self.landing_zone_col.find(myquery)
        for document in cursor:
            #print(document['Zone Number'])
            open_zone_names.append(document['Zone Number'])
            open_zone_coordinates.append(tuple(document['location']))

        return open_zone_names, open_zone_coordinates

    def get_uavs(self):
        myquery = {"landing_service_status": 0}
        uav_names = []
        uav_battery = []
        cursor = self.landing_service_col.find(myquery)
        for document in cursor: 
            uav_names.append(document['uav_name'])
            uav_battery.append(document['battery'])
           
        return uav_names, uav_battery

    def get_uav_location(self, uav_list):
        uav_locations = []
        for uav_name in uav_list:
            for item, meta in self.data_srv_col_prox.query_named(uav_name, StringPairList._type, single=False):
                msg_type = item.pairs[1].first 
                msg_id = item.pairs[1].second
                pose = self.data_srv_col_prox.query_id(msg_id, msg_type)[0].pose
                x = pose.position.longitude
                y = pose.position.latitude
                z = pose.position.altitude
                locs = [x,y]
                uav_locations.append(locs)

        return uav_locations           

    def find_closest_zone(self, uav_loc, landing_zones):
        """find closest zone index to uav location"""
        tree = spatial.KDTree(landing_zones)
        dist,zone_index = tree.query(uav_loc)

        return dist, zone_index

    def assign_uav_zone(self,uav_name, zone_name):
        """assigns uav to zone and sets the landing zone as false, so no longer vacant"""
        self.landing_zone_col.update({"Zone Number": zone_name},
            { "$set": { 
                "Occupied by": uav_name,
                "Vacant": False }})

        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "Zone Assignment": zone_name }})

        print("Assigned", uav_name + " to landing zone ", zone_name)


if __name__ == '__main__':
    rospy.init_node("uav_sending_info")
    preLandingService = PreLandingService()
    
    if preLandingService.check_open_zones() == False:
        print("No open zones")
    else:
        uav_names, uav_battery = preLandingService.get_uavs()
        uav_loc_list = preLandingService.get_uav_location(uav_names)
        for idx, uav_loc in enumerate(uav_loc_list): 
            zone_names, zone_coordinates = preLandingService.find_open_zones()
            print("uav location", uav_loc)
            dist, zone_idx = preLandingService.find_closest_zone(uav_loc, zone_coordinates)
            preLandingService.assign_uav_zone(uav_names[idx], zone_names[zone_idx])

        
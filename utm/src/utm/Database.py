#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import pymongo
import json
import numpy as np

from utm import UAVGen
from mongodb_store_msgs.msg import StringPairList, StringPair
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool, Int32
from datetime import *
import platform

if float(platform.python_version()[0:2]) >= 3.0:
    import io
else:
    import StringIO

class AbstractDatabaseInfo():
    """Abstract Database distrubutes information of database 
    through querys"""
    def __init__(self, ip, portnumber, poolsize):
        #self.msg_store = MessageStoreProxy(collection= collection_name)
        self.client = pymongo.MongoClient(ip, portnumber, maxPoolSize = poolsize)
    
    def does_object_exist(self):
        """check if key item exists in database returns True if it does, False if not"""

    def access_database(self, database_name):
        """reurn database based on string:database_name"""
        database = self.client[database_name]

        return database 
          
    def access_collection(self,database, collection_name):
        """return collection name from """
        collection = database[collection_name]

        return collection 

    def unwrap_info(self, ros_col_prox, meta_info, object_name):
        """unwrap stringpair list from rosmessageproxy, and the object_name in the meta information"""
        id_list = []
        msg_info = []
        i = 0
        for item,meta in ros_col_prox.query_named(object_name, StringPairList._type, single=False):
            print(item.pairs[i].first)
            msg_info.append(item.pairs[i].first)
            id_list.append(item.pairs[i].second)

        return id_list, msg_info
          
class AbstractDatabaseSend():
    """AbstractDatabaseSend allows writing to database
    Keyword arguments:
    string: name -- name used to send to Mongodb, needs to be a string
    string: collection_name -- collection name to access information from 
    """
    def __init__(self, collection_name):
        self.msg_store = MessageStoreProxy(collection= collection_name)

    def update_db_info(self):
        """update databse"""

    def remove_from_db(self):
        """removes from database based on some condition"""

    def add_to_db(self, spl, meta):
        """adds the nonexistent item and its information
        uses helper functions, wrap_database_info, combine_info_ids,
        and generate_meta_info"""
        self.msg_store.insert(spl, meta)

    def wrap_database_info(self,msg_list):
        """takes in msg_list which consist of ros messages"""        
        stored = [] 
        for msg in msg_list:
            stored.append([msg._type, self.msg_store.insert(msg)])

        return stored

    def combine_info_ids(self, stored):
        spl = StringPairList()
        for pair in stored:
            spl.pairs.append(StringPair(pair[0], pair[1]))
        return spl

    def generate_meta_info(self,name):   
        "name must be type of string"
        meta = {}
        meta['name'] = name
        meta['result_time'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
        return meta

class ZonePlanner():
    """
    Helps the pathway, landing, and postflight nodes with respective information about
    uavs and their assigned Database
    """
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "message_store"
    landing_srv_col_name = "landing_service_db"
    landing_zone_col_name = "landing_zones"

    def __init__(self):
        #super().__init__()
        #access database
        
        self.dbInfo = AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        #recieve collection information
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
        self.landing_zone_col = self.mainDB[self.landing_zone_col_name]

        #ros proxy messages
        self.control_dict = {}

    def find_assigned_zones(self, service_num):
        """check if uav has an assigned location and get information 
        based on a state service"""
        #myquery = {"Zone Assignment": {"$exists": True}}
        uavs = []
        zones = []
        myquery = {"$and": [{"landing_service_status":service_num}, 
                    {"Zone Assignment": {'$exists': True}}]}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uavs.append(document["uav_name"])
            zones.append(document["Zone Assignment"])

        return uavs,zones

    def find_uav_homeposition(self, uav_name):
        myquery = {"uav_name" : uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            home_position = document["Home Position"]

        return home_position

    def find_uav_waypoints(self, uav_name):
        """request uav waypoint from database"""
        myquery = {"_id": uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uav_waypoints = document["Waypoint"]

        return uav_waypoints     

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

    def is_arrived_to_zone(self, zone_coords, uav_coords):
        zone_coords = np.array(zone_coords)
        uav_coords = np.array(uav_coords)
        #print("uav", uav_coords)
        #print("zone", zone_coords)
        dist = (np.sqrt((zone_coords[0]- uav_coords[0])**2+(zone_coords[1]- uav_coords[1])**2))
        #print(dist)
        #print("dist", dist)
        if dist <= 1.5:
            return True
        else:
            return False

    def has_left_zone(self, zone_coords, uav_coords):
        zone_coords = np.array(zone_coords)
        uav_coords = np.array(uav_coords)
        #print(uav_coords)
        dist = abs(np.sqrt((zone_coords[0]- uav_coords[0])**2+(zone_coords[1]- uav_coords[1])**2))
        #print(dist)
        #print(dist)
        if dist >= 0.25:
            return True
        else:
            return False

    def update_uav_state(self, uav_name, new_status):
        self.landing_service_col.update({"uav_name": uav_name},
        {"$set":{
            "landing_service_status": new_status
        }})

    def update_landing_zone(self, zone_number):
        self.landing_zone_col.update({"_id": zone_number},
        {"$set":{
            "Vacant": True
        }})
        print("Landing ", zone_number + " is now vacant")

    def remove_uav(self, uav_name):
        """remove uav from landing service collection """
        myquery = {"uav_name": uav_name}
        self.landing_service_col.delete_one(myquery)

        print("Removed ", uav_name + " from landing service collection")
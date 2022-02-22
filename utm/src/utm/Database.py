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
    """Abstract Database requests information of database 
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
    """AbstractDatabaseSend allows writing to database as ROS protocol
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
            home_position = document["uav_home"]

        return home_position

    def find_uav_loiter_position(self, uav_name):
        myquery = {"uav_name" : uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            home_position = document["uav_location"]

        return home_position

    def find_uav_info(self, uav_name, field_name):
        """return uav information based on uav name and specific
        field name this can probably replace the other functions up top"""
        myquery = {"uav_name" : uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uav_info = document[field_name]

        return uav_info

    def find_uav_zone_loc(self, uav_name):
        """query where the uav was located at"""
        myquery = {"Occupied by": uav_name}
        cursor = self.landing_zone_col.find(myquery)
        for document in cursor:
            zone_loc = document["location"]

        return zone_loc

    def find_uav_zone_name(self, uav_name):
        myquery = {"uav_name" : uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            zone_assignment = document["Zone Assignment"]

        return zone_assignment

    def find_uav_waypoints(self, uav_name):
        """request uav waypoint from database"""
        myquery = {"_id": uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uav_waypoints = document["Waypoint"]

        return uav_waypoints

    def find_uav_home_waypoints(self, uav_name):
        """request uav waypoint from database"""
        myquery = {"_id": uav_name}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uav_waypoints = document["Home Waypoints"]

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

    def is_arrived_to_zone(self, zone_coords, uav_coords, tolerance):
        zone_coords = np.array(zone_coords)
        uav_coords = np.array(uav_coords)
        dist = (np.sqrt((zone_coords[0]- uav_coords[0])**2+(zone_coords[1]- uav_coords[1])**2))
        if dist <= tolerance:
            return True
        else:
            return False

    def has_left_zone(self, zone_coords, uav_coords):
        """check if uav has left zone in the long and lat distance"""
        zone_coords = np.array(zone_coords)
        uav_coords = np.array(uav_coords)
        dist = abs(np.sqrt((zone_coords[0]- uav_coords[0])**2+(zone_coords[1]- uav_coords[1])**2))
        if dist >= 15:
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


class PathPlannerService(AbstractDatabaseInfo):
    """
    Path Planner Service takes in all information of UAS queries to request 
    a path to be sent from start point to goal point

    """
    database_name = "pathPlanningService"
    path_planning_col_name = "uas_path_planning"
    reservation_col_name = "reservation_table"

    ip = "127.0.0.1"
    port_num = 27017
    poolsize = 100

    def __init__(self):
        super().__init__(self.ip,self.port_num,self.poolsize)
        self.mainDB = self.access_database(self.database_name)  

        #collection names
        self.path_planning_col = self.mainDB[self.path_planning_col_name]
        self.reservation_col = self.mainDB[self.reservation_col_name]

    def request_path(self,uav_name, start_point, end_point):
        """request path to db"""
        uav_info = {
                    "uav_name": uav_name,
                    "start_point": start_point,
                    "end_point": end_point
                    }

        self.path_planning_col.insert(uav_info)
                   
    def check_waypoints_exist(self,uav_name,start,goal):
        """check if waypoints exist in database returns true if it does
        false if it doesnt"""
        myquery = {"$and": [{"uav_name":uav_name, 
                    "start_point":start,"end_point":goal}, 
                    {"waypoints": {'$exists': True}}]}
        #myquery = {"waypoints": {'$exists': True}}
        cursor = self.path_planning_col.find(myquery)
        if len(list(cursor)) == 0:
            return False
        else:
            return True
            
    def find_path_planning_clients(self):
        """find uas operators who do not have a waypoint and returns as list of lists"""
        uavs = []
        myquery = {"waypoints": {'$exists': False}}
        cursor = self.path_planning_col.find(myquery)
        for document in cursor:
            uav = [document["uav_name"], document["start_point"],
            document["end_point"]]
            uavs.append(uav)

        return uavs
    
    def get_uav_waypoints(self, uav_name, start, goal):
        myquery = {"$and": [{"uav_name":uav_name, 
                    "start_point":start,"end_point":goal}, 
                    {"waypoints": {'$exists': True}}]}
        #myquery = {"waypoints": {'$exists': True}}
        cursor = self.path_planning_col.find(myquery)
        for document in cursor:
            return document["waypoints"]

    def prioritize_uas(self,uav_list):
        """Takes in start list, and goal list and 
        prioritizes UAS based on highest distance to be traversed"""
        
        dist_list = []
        for uav in uav_list:
            dist_val = compute_actual_euclidean(uav[1], uav[2])
            dist_list.append((dist_val,uav[1], uav[2], uav[0]))

        ##setting reverse to false sets to min first, true max first
        final_list = sorted(dist_list, key=lambda x: x[0], reverse=True)
        sorted_start = [start[1] for i, start in enumerate(final_list)]
        sorted_goal = [goal[2] for i, goal in enumerate(final_list)]
        sorted_uavs = [uav_name[3] for i, uav_name in enumerate(final_list)]

        return final_list, sorted_start, sorted_goal, sorted_uavs

    def insert_waypoints(self, uav_name, waypoint_list):
        """insert waypoints into path planning collection based on 
        uav name"""
        self.path_planning_col.update({"uav_name": uav_name,
        "start_point": waypoint_list[0], "end_point":waypoint_list[-1]},
        {"$set":{
            "waypoints": waypoint_list}})

    def remove_uav_from_reservation(self, uav_name):
        """remove uav from collection list"""
        myquery = {"uav_name": uav_name}
        self.reservation_col.delete_one(myquery)
        #print("Removed ", uav_name + "from reservation table")

    def insert_uav_to_reservation(self,uav_name, waypoints):
        """insert uav into reservation collection"""
        uav_info = {
                    "uav_name": uav_name,
                    "waypoints": waypoints,
                    }
        self.reservation_col.insert(uav_info)

    def get_reserved_waypoints(self):
        """find uas operators who do not have a waypoint and returns as list of lists"""
        reserved = []
        myquery = {"waypoints": {'$exists': True}}
        cursor = self.reservation_col.find(myquery)
        for document in cursor:
            reserved.append(document["waypoints"][0])
        return reserved
    
def compute_actual_euclidean(position, goal):
    distance =  (((position[0] - goal[0]) ** 2) + 
                        ((position[1] - goal[1]) ** 2) +
                        ((position[2] - goal[2]) ** 2))**(1/2)
    
    return distance

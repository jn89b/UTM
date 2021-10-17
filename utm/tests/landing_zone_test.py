#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util

from utm import Database
from mongodb_store_msgs.msg import StringPairList, StringPair
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool
from datetime import *
import platform


if float(platform.python_version()[0:2]) >= 3.0:
    import io
else:
    import StringIO

def generate_landing_zones(num_zones):
    """"""
    zone_name_list = []

    for zone in list(range(0,num_zones)): 
        zone_name_list.append("Zone" + " " + str(zone))

    print(zone_name_list)    
    return zone_name_list

def wrap_database_info(pose_msg, bool_msg, msg_store):
    """pose_msg and bool_msg information are wrapped"""
    stored = []
    stored.append([pose_msg._type, msg_store.insert(pose_msg)])
    stored.append([bool_msg._type, msg_store.insert(bool_msg)])
    
    return stored

def combine_info_ids(stored):
    spl = StringPairList()
    for pair in stored:
        spl.pairs.append(StringPair(pair[0], pair[1]))
    return spl

def generate_meta_info(name):
    "name must be type of string"
    meta = {}
    meta['name'] = name
    meta['result_time'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
    return meta

def insert_to_landing_zones_col(collection,zone, location, open_close):
    """insert to landing collection the uav name, battery, and state of service"""
    post = {"_id": zone,
            "Zone Number": zone,
            "location" : location,
            "Vacant": open_close
    }
    collection.insert_one(post)
    #print(zone + " " + "added to database\n")

if __name__ == '__main__':
    """
    input number of zones 
    assign positions for zones
    initialize zone as vacant
    send to database
    """
    rospy.init_node("landing_zone_db")
    
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "message_store"
    landing_zone_col_name = "landing_zones"

    dbInfo = Database.AbstractDatabaseInfo(ip_address, port_num, poolsize)
    mainDB = dbInfo.access_database(database_name)
    landing_collection = mainDB[landing_zone_col_name]

    num_zones = 4
    """need to map this to landing zone bases"""
    p_0 = Pose(Point(0, 1, 0), Quaternion(0,0,0,0))
    p_1 = Pose(Point(0, 4, 0), Quaternion(0,0,0,0))
    p_2 = Pose(Point(1, 0, 0), Quaternion(0,0,0,0))
    p_3 = Pose(Point(4, 0, 0), Quaternion(0,0,0,0))
    p_list = [p_0, p_1, p_2, p_3]
    isvacant = Bool(False)

    msg_store = MessageStoreProxy(collection='landing_zones')
    zone_names_list = generate_landing_zones(num_zones=num_zones)

    try:
        """
        loop through and append zone locations and names to database
        database should be n x 3 
        where n is the number of landing zones
        3 is the documents for:
            -pose
            -is zone vacant or empty 
            -string pair list of id 
        """
        #map this with airsim location of where the apriltags are 
        insert_to_landing_zones_col(landing_collection, "Zone_0", [0, 4.5], True)
        insert_to_landing_zones_col(landing_collection, "Zone_1", [5.7, 4.1], True)
        insert_to_landing_zones_col(landing_collection, "Zone_2", [0, 0], False)
        #insert_to_landing_zones_col(landing_collection, "Zone_3", [-10, -4], True)
        """
        for index, zone in enumerate(zone_names_list):
            stored = wrap_database_info(p_list[index], isvacant, msg_store)
            spl = combine_info_ids(stored)
            meta_info = generate_meta_info(zone)
            msg_store.insert(spl,meta_info) 
        """
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


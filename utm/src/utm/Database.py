#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import pymongo
import json

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

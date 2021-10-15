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
    def __init__(self, name, collection_name):
        self.name = name
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

    def wrap_database_info(self, pose_msg, bool_msg):
        """pose_msg and bool_msg information are wrapped
        needs to be customizable for multiple messages"""        
        stored = []
        stored.append([pose_msg._type, self.msg_store.insert(pose_msg)])
        stored.append([bool_msg._type, self.msg_store.insert(bool_msg)])
        
        return stored

    def combine_info_ids(self, stored):
        spl = StringPairList()
        for pair in stored:
            spl.pairs.append(StringPair(pair[0], pair[1]))
        return spl

    def generate_meta_info(self):   
        "name must be type of string"
        meta = {}
        meta['name'] = self.name
        meta['result_time'] = datetime.utcfromtimestamp(rospy.get_rostime().to_sec())
        return meta

        
class LandingDBNode():
    """
    LandingDB appends all UAVs that are requesting the third party landing service
    assigns initial state of UAV as 0

    Attributes
    dbInfo : AbstractDatabaseInfo()
        Instiantes abstract databaseinfo:
        ip_address : str
        port_num : int
        poolsize : int
    
    
    """
    database_name = "message_store"
    main_col_name = "data_service"
    landing_col_name = "landing_service_db"

    def __init__(self, ip_address, port_num,poolsize):
        
        #access database
        self.dbInfo= AbstractDatabaseInfo(ip_address, port_num, poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)
    
        #mongodb
        self.main_collection = self.get_collection(self.mainDB, self.main_col_name)      
        self.landing_collection = self.get_collection(self.mainDB, self.landing_col_name)

        #ros service proxies with mongodb
        self.data_srv_col_prox = MessageStoreProxy(collection=self.main_col_name)    
        self.landing_srv_col_prox = MessageStoreProxy(collection= self.landing_col_name)

        #self.dataBaseInfo.retrieve_all_objects()
        self.sub = None

    @classmethod
    def get_collection(self,database, col_name):
        """class method that returns collection from mongodb database input and collection name"""
        collection = database[col_name]
        return collection

    def main(self):
        """main function implementation"""
        rate = rospy.Rate(0.25)
        while not rospy.is_shutdown():
            self.listen_for_incoming()
            rate.sleep()

    def listen_for_incoming(self):
        """listen for any incoming objects"""
        myquery = {"pairs": {"$exists": True}}

        for doc in self.main_collection.find(myquery):
            meta_info = doc['_meta']
            uav_name = meta_info['name']
            bat_val = self.get_uav_battery_info(uav_name=uav_name)

            if (self.get_uav_srv_info(uav_name) == False) or (self.does_uav_exist(uav_name) == True):
                continue

            if (self.is_landing_collection_empty()== True) or (self.does_uav_exist(uav_name) == False):
                self.insert_to_landing_collection(uav_name, bat_val, 0)

    def get_uav_srv_info(self, uav_name):
        """get uav service request info"""
        for item,meta in self.data_srv_col_prox.query_named(uav_name, StringPairList._type, single=False):
            srv_msg_type = item.pairs[3].first 
            srv_id = item.pairs[3].second
            srv_val = self.data_srv_col_prox.query_id(srv_id, srv_msg_type)[0].data
            if srv_val == False:
                print(uav_name + " " + "does not want the service\n")
            return srv_val

    def get_uav_battery_info(self, uav_name):
        """get battery information"""
        for item,meta in self.data_srv_col_prox.query_named(uav_name, StringPairList._type, single=False):
            batter_msg_type = item.pairs[0].first 
            battery_id = item.pairs[0].second
            battery_val = self.data_srv_col_prox.query_id(battery_id, batter_msg_type)[0].data

            return battery_val

    def insert_to_landing_collection(self, uav_name, battery_val, state_val):
        """insert to landing collection the uav name, battery, and state of service"""
        post = {"_id": uav_name,
                "uav_name": uav_name,
                "battery" : battery_val,
                "landing_service_status": state_val
        }
        self.landing_collection.insert_one(post)
        print(uav_name + " " + "added to database\n")
    
    def is_landing_collection_empty(self):
        """checks if collection has field name 
        field name is type string
        return True if it does"""
        #myquery = {field_name: {"$exists": True}} 
        if self.landing_collection.count(()) == 0:
            return True

    def does_uav_exist(self,uav_name):
        """check if uav key exists in this database"""
        myquery = {"_id": uav_name}
        cursor = self.landing_collection.find(myquery)
        if cursor.count() == 0: #means no uav is in there
            return False
        else:
            print(uav_name + " " + "exists already in database\n")
        
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
    
    landingDBNode  = LandingDBNode(ip_address=ip_address, port_num=port_num, poolsize=poolsize)
    srv_collection = landingDBNode.main_collection
    landing_srv_collection = landingDBNode.landing_collection

    try:
        #landingDBNode.listen_for_incoming()
        landingDBNode.main()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


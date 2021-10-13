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

        
class UTMDB():
    def __init__(self):
        self.dataBaseInfo = AbstractDatabaseInfo()
        self.dataBaseInfo.retrieve_all_objects()
        self.sub = None

    def listen_for_incoming(self):
        """listen for any incoming objects"""
        pass

    def main(self):
        """main function implementation"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.sleep(rate)

def does_uav_exist(uav_name, collection):
    myquery = {"_id": uav_name}
    cursor = collection.find(myquery)
    if cursor.count() == 0: #means no uav is in there
        return False

def is_collection_empty(collection):
    """checks if collection has field name 
    field name is type string
    return True if it does"""
    #myquery = {field_name: {"$exists": True}} 
    if collection.count(()) == 0:
        return True
        

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
    data_srv_col_prox = MessageStoreProxy(collection='data_service')    
    landing_srv_col_prox = MessageStoreProxy(collection='landing_service_db')

    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    
    mainDB = AbstractDatabaseInfo(ip_address, port_num, poolsize)
    db = mainDB.access_database('message_store')
    srv_collection = db['data_service'] #go to respective collections
    landing_srv_collection = db['landing_service_db']

    try:
        myquery = {"pairs": {"$exists": True}} # {"field name": {"$condition": True}} 
        """loop through retrieve meta information
        check name of drone and see if its requesting service"""
        for doc in srv_collection.find(myquery):
            #print("doc is", doc)
            #battery_life = doc['battery']
            meta_info = doc['_meta']
            uav_name = meta_info['name']
            print("checking", uav_name)
            #check if uav wants to use the service
            for item,meta in data_srv_col_prox.query_named(uav_name, StringPairList._type, single=False):
                srv_id = item.pairs[3].second
                srv_val = data_srv_col_prox.query_id(srv_id, Bool._type)[0].data
                print("service val is:",srv_val)

                #if the uav does not want the service then we ignore
                if srv_val == False:
                    print(uav_name + " " + "does not want the service\n")
                    continue

                battery_id = item.pairs[0].second
                battery_val = data_srv_col_prox.query_id(battery_id, Int32._type)[0].data

                #if collection is empty we slap on that first drone if its requesting info
                if is_collection_empty(landing_srv_collection):
                    post = {"_id": uav_name,
                            "uav_name": uav_name,
                            "battery" : battery_val,
                            "landing_service_status": 0
                    }
                    landing_srv_collection.insert_one(post)
                    print(uav_name + " " + "added to database\n")
                
                else:
                    print("database is not empty")
                    #check if uav is already in database
                    if does_uav_exist(uav_name, landing_srv_collection) == False:
                        post = {"_id": uav_name,
                                "uav_name": uav_name,
                                "battery" : battery_val,
                                "landing_service_status": 0
                        }
                        landing_srv_collection.insert_one(post)
                        print(uav_name + " " + "added to database\n")
                        
                    else:
                        print(uav_name + " " + "exists already in database\n")


    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


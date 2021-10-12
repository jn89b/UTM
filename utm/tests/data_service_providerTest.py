#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import threading

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

class AbstractDatabaseInfo():
    """Abstract Database distrubutes information of database 
    through querys"""
    def __init__(self):
        self.sub = None

    def does_object_exist(self):
        """check if key item exists in database returns True if it does, False if not"""
        
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
        self.dataBase = AbstractDatabaseInfo()
        self.sub = None

    def listen_for_incoming(self):
        """listen for any incoming objects"""
        pass

    def main(self):
        """main function implementation"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rospy.sleep(rate)

class TestUAV():
    """this is a test module for UAV to send the respective information to the DataServiceDB"""
    def __init__(self, name,  position, quaternion, service_request):
        self.name = name  #string
        self.pose_msg = self.get_pose_msg(position, quaternion)
        self.srv_req = self.get_bool_msg(service_request) #boolean
        self.dataService = AbstractDatabaseSend(name, "data_service")
        self.send_information()

    def get_pose_msg(self, pos,quat):
        pose_msg = Pose(Point(pos[0], pos[1], pos[2]) ,Quaternion(quat[0],quat[1], quat[2],quat[3]))
        print("pose is:" , pose_msg)
        return pose_msg

    def get_bool_msg(self,bool_statement):
        bool_msg = Bool(bool_statement)
        return bool_msg

    def send_information(self):
        """send information to dataservirve"""
        self.stored = self.dataService.wrap_database_info(self.pose_msg, self.srv_req)
        self.spl = self.dataService.combine_info_ids(self.stored)
        self.meta = self.dataService.generate_meta_info()
        self.dataService.add_to_db(self.spl, self.meta)
    
if __name__ == '__main__':

    rospy.init_node("uav_sending_info")
    try:
        uav_0 = TestUAV("uav0", [0,1,2], [0,0,0,1], True)
        uav_1 = TestUAV("uav1", [0,1,2], [0,0,0,1], True)
        uav_2 = TestUAV("uav2",  [0,1,2],[0,0,0,1], False)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


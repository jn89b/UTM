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

from datetime import *
import platform

from geometry_msgs.msg import Pose, Point, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Bool, Int32


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
    string: collection_name -- collection name to access information from 
    """
    def __init__(self, collection_name):
        self.msg_store = MessageStoreProxy(collection= collection_name)

    def update_db_info(self):
        """update databse"""

    def remove_from_db(self):
        """removes from database based on some condition"""

    def add_to_db(self, doc_info, meta):
        """adds the nonexistent item and its information
        uses helper functions, wrap_database_info, combine_info_ids,
        and generate_meta_info"""
        self.msg_store.insert(doc_info, meta)

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

class TestUAV():
    """this is a test module for UAV to send the respective information to the DataServiceDB
        -uav battery -- Int32 
        -uav position information global reference(pose and quat)
        -uav waypoint waypoint position as tuple(pose and quat)
        -uav service request -- Bool
        -uav state -- String
    """
    def __init__(self, name, current_pose, wp_dest, battery, service_request):
        self.name = name  #string
        
        """for these get messages it will be using ROS's callback functions"""
        self.srv_req = self.get_bool_msg(service_request) #boolean
        self.battery_msg = self.get_battery_msg(battery)
        self.current_pose = self.get_lat_long_msg(current_pose)
        self.wp_dest = self.get_lat_long_msg(wp_dest)
        
        self.dataService = AbstractDatabaseSend("data_service")
        self.send_information()

    def get_bool_msg(self,bool_statement):
        bool_msg = Bool(bool_statement)
        return bool_msg

    def get_battery_msg(self,battery):
        battery_msg = Int32(battery)
        return battery_msg

    def get_lat_long_msg(self, lat_long):
        position = lat_long[0]
        quat = lat_long[1]
        lat_long_msg = GeoPoseStamped()
        lat_long_msg.pose.position.latitude = position[0]
        lat_long_msg.pose.position.longitude = position[1]
        lat_long_msg.pose.position.altitude = position[2]
        lat_long_msg.pose.orientation.x = quat[0]
        lat_long_msg.pose.orientation.y = quat[1]
        lat_long_msg.pose.orientation.z = quat[2]
        lat_long_msg.pose.orientation.w = quat[3]
        return lat_long_msg        

    def send_information(self):
        """send information to dataservice need to check if uav is already in database 
        if so we just update the respective information such as position, battery life,etc"""
        msg_list = [self.battery_msg,self.current_pose, self.wp_dest, self.srv_req]
        self.stored = self.dataService.wrap_database_info(msg_list)
        self.spl = self.dataService.combine_info_ids(self.stored)
        self.meta = self.dataService.generate_meta_info(self.name)
        self.dataService.add_to_db(self.spl, self.meta)
    
if __name__ == '__main__':

    rospy.init_node("uav_sending_info")
    try:
        #uav_0 = TestUAV("uav0",[[47.65, -122.14015, 100],[0,0,0,1]], [[50, -123, 100],[0,0,0,1]], 82,False)
        #uav_1 = TestUAV("uav1",[[47.65, -122.14015, 100],[0,0,0,1]], [[50, -123, 100],[0,0,0,1]], 100,True)
        #uav_2 = TestUAV("uav2",[[47.65, -122.14015, 100],[0,0,0,1]], [[50, -123, 100],[0,0,0,1]], 55,False)
        #uav_3 = TestUAV("uav3",[[47.65, -122.14015, 100],[0,0,0,1]], [[50, -123, 100],[0,0,0,1]], 55,True)
        #uav_4 = TestUAV("uav4",[[47.65, -122.14015, 100],[0,0,0,1]], [[50, -123, 100],[0,0,0,1]], 55,True)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


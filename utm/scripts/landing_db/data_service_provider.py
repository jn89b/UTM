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

from utm import Database
from geometry_msgs.msg import Pose, Point, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Bool, Int32


if float(platform.python_version()[0:2]) >= 3.0:
    import io
else:
    import StringIO

class TestUAV():
    """this is a test module for UAV to send the respective information to the DataServiceDB
        -uav battery -- Int32 
        -uav home position information global refererence()
        -uav current position information global reference(pose and quat)
        -uav waypoint waypoint position as tuple(pose and quat)
        -uav service request -- Bool
        -uav state -- String
    """
    def __init__(self, name, home_position, current_pose, wp_dest, battery, service_request):
        self.name = name  #string
        
        """for these get messages it will be using ROS's callback functions"""
        self.battery_msg = self.get_battery_msg(battery)
        self.home_position = self.get_lat_long_msg(home_position)
        self.current_pose = self.get_lat_long_msg(current_pose)
        self.wp_dest = self.get_lat_long_msg(wp_dest)
        self.srv_req = self.get_bool_msg(service_request) #boolean
        
        self.dataService = Database.AbstractDatabaseSend("data_service")
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
        msg_list = [self.battery_msg, self.home_position, self.current_pose, self.wp_dest, self.srv_req]
        self.stored = self.dataService.wrap_database_info(msg_list)
        self.spl = self.dataService.combine_info_ids(self.stored)
        self.meta = self.dataService.generate_meta_info(self.name)
        self.dataService.add_to_db(self.spl, self.meta)
    
if __name__ == '__main__':

    rospy.init_node("uav_sending_info")
    try:
        uav_0 = TestUAV("uav0",[[0, 5, 10],[0,0,0,1]],[[3, 0, 10],[0,0,0,1]], [[10, -123, 100],[0,0,0,1]], 82,True)
        uav_1 = TestUAV("uav1",[[5, 0, 10],[0,0,0,1]],[[0, 10, 10],[0,0,0,1]], [[10, -123, 100],[0,0,0,1]], 82,True)
        uav_2 = TestUAV("uav2",[[10, 20, 10],[0,0,0,1]],[[3, 0, 10],[0,0,0,1]], [[10, -123, 100],[0,0,0,1]], 82,False)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


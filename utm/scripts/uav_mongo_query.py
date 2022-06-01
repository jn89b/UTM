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

from utm import Database
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import Bool, Int32


"""
Data that listens for all incoming uavs id 
If does not exist we will dump them in the overall database 
else we will just update the information of the uavs
"""

class UAV():
    """this is a test module for UAV to send the respective information to the DataServiceDB
        -uav battery -- Int32 
        -uav position information global reference(pose and quat)
        -uav waypoint waypoint position as tuple(pose and quat)
        -uav service request -- Bool
        -uav state -- String
    """
    def __init__(self, name, battery, service_request):
        self.name = name  #string
        
        """for these get messages it will be using ROS's callback functions"""
        self.srv_req = self.get_bool_msg(service_request) #boolean
        self.battery_msg = self.get_battery_msg(battery)
        self.dataService = Database.AbstractDatabaseSend("data_service")
        #self.send_information()

        self.position_topic_name = self.name + "mavros/offset_local_position/pose"
        self.position_sub = rospy.Subscriber(self.position_topic_name, PoseStamped, self.position_cb)

        self.waypoint_topic_name = self.name +"mavros/setpoint_position/local"
        self.waypoint_sub = rospy.Subscriber(self.waypoint_topic_name, PoseStamped, self.waypoint_cb)

        self.current_pose = None
        self.wp_dest = None

    def position_cb(self, msg):
        """position callback for my function"""
        position = msg.pose.position
        orientation = msg.pose.orientation

        position_vector  = [position.x, position.y, position.z]
        orientation_vector = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.current_pose = self.get_lat_long_msg([position_vector, orientation_vector])

    def waypoint_cb(self, msg):
        """waypoint callback function"""
        position = msg.pose.position
        orientation = msg.pose.orientation

        position_vector  = [position.x, position.y, position.z]
        orientation_vector = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.wp_dest = self.get_lat_long_msg([position_vector, orientation_vector])

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
        #print("message list", msg_list)
        self.stored = self.dataService.wrap_database_info(msg_list)
        self.spl = self.dataService.combine_info_ids(self.stored)
        self.meta = self.dataService.generate_meta_info(self.name)
        self.dataService.add_to_db(self.spl, self.meta)

    def check_message_valid(self):
        if self.current_pose == None or self.wp_dest == None:
            return False

if __name__ == '__main__':
    rospy.init_node("uav_sending_info")
    rate = rospy.Rate(0.01)
    uav_id = rospy.get_param("~uav_name", "")
    uav = UAV(uav_id, 82, True)
    while not rospy.is_shutdown():
        if uav.check_message_valid() == False:
            print("invalid message")
            continue
        else:
            print("valid message")
            uav.send_information()
        
        rate.sleep() 

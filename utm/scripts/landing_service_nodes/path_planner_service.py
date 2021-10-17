#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import threading
from utm import Database
from scipy import spatial

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from mongodb_store_msgs.msg import StringPairList
from mongodb_store.message_store import MessageStoreProxy
from datetime import *

class UAVComms():
    """uav communication protocol"""
    def __init__(self,name):
        self.name = name
        self.uav_pos_pub = self.generate_uav_pos_command(self.name)
        self.uav_state_cmd_pub = self.generate_uav_state_command(self.name)
        #self.uav_loc_sub = self.generate_uav_loc(self.name)

    def generate_uav_pos_command(self, uav_name):
        """generate a ros publisher with str:uav name"""
        topic_name = uav_name+"/utm/mavros/setpoint_position/local"
        uav_pos_pub = rospy.Publisher(topic_name, PoseStamped, queue_size = 10)
        return uav_pos_pub

    def send_waypoint_command(self, wp_coords):
        now = rospy.Time.now()
        wp_msg = PoseStamped()
        wp_msg.header.frame_id = str(self.name)
        wp_msg.header.stamp = now
        wp_msg.pose.position.x = wp_coords[0]
        wp_msg.pose.position.y = wp_coords[1]
        wp_msg.pose.position.z = 5
        print(wp_msg)
        self.uav_pos_pub.publish(wp_msg)

        state_cmd_msg = Int8()
        state_cmd_msg.data = 0
        self.uav_state_cmd_pub.publish(state_cmd_msg)

    def generate_uav_state_command(self, uav_name):
        """generate a ros publisher with str:uav name"""
        topic_name = uav_name+"/utm_control"
        uav_state_cmd_pub = rospy.Publisher(topic_name, Int8, queue_size = 10)
        print(uav_state_cmd_pub)
        return uav_state_cmd_pub


class PathPlanner():
    """
    Look for drones at state 0 and have an assigned waypoint from waypoint collections
    Plan their path trajectories based on where they are at and their current waypoint
    Open up multiple threads to send location waypoints to these drones 
    Once at goal waypoint set UAV state to 1
    """
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "message_store"
    landing_srv_col_name = "landing_service_db"
    landing_zone_col_name = "landing_zones"

    def __init__(self):
        
        #access database
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        #recieve collection information
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
        self.landing_zone_col = self.mainDB[self.landing_zone_col_name]

        #ros proxy messages
        self.control_dict = {}

    def find_assigned_zones(self):
        """check if uav has an assigned location and if they are at state 0"""
        #myquery = {"Zone Assignment": {"$exists": True}}
        uavs = []
        zones = []
        myquery = {"$and": [{"landing_service_status": 0}, 
                    {"Zone Assignment": {'$exists': True}}]}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uavs.append(document["uav_name"])
            zones.append(document["Zone Assignment"])

        return uavs,zones

    def find_zone_waypoints(self, zone_number):
        myquery = {"Zone Number": zone_number}
        cursor = self.landing_zone_col.find(myquery)
        for document in cursor:
            zone_coordinates = document["location"]

        return zone_coordinates

    def get_zone_wp_list(self, zone_names):
        zone_coords = []
        for zone in zone_names:
            zone_coords.append(pathPlanner.find_zone_waypoints(zone))
        
        return zone_coords

    def plan_uav_path(self):
        """takes in uav"""
        pass

    def send_wp_cmds(self):
        """open up multiple threads to send waypoints to drones"""

    def generate_publishers(self, uavs):
        uavObject_list = []
        for uav in uavs:
            uavComms = UAVComms(uav)
            uavObject_list.append(uavComms)
        
        return uavObject_list
    

if __name__ == '__main__':
    rospy.init_node('utm_path_planner')
    pathPlanner = PathPlanner()
    uavs,zone_names = pathPlanner.find_assigned_zones()
    zone_coord_list = pathPlanner.get_zone_wp_list(zone_names)
    uav_class_list = pathPlanner.generate_publishers(uavs)

    """generate publishers and begin sending waypoint commands to drones""" 
    threads = []
    #open multiple threads to begin publising the waypoint command to the drone

    rate_val = 10
    rate = rospy.Rate(rate_val)

    while not rospy.is_shutdown():
        for i in range(len(uav_class_list)):
            #print(uav_class_list[i].uav_pos_pub)
            #uav_class_list[i].send_waypoint_command(zone_coord_list[i])
            t = threading.Thread(target=uav_class_list[i].send_waypoint_command(zone_coord_list[i]), args=(i,))
            t.start()
            threads.append(t)
        #rospy.spin()
        rate.sleep()





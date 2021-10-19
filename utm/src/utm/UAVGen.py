#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

class UAVComms():
    """uav communication protocol"""
    def __init__(self,name):
        self.name = name
        self.uav_pos_pub = self.generate_uav_pos_command(self.name)
        self.uav_state_cmd_pub = self.generate_uav_state_command(self.name)
        self.uav_loc_sub = self.generate_uav_position_sub(self.name)
        self.coords = [None,None]
        self.three_cords = [None,None,None]

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

    def generate_uav_state_command(self, uav_name):
        """generate a ros publisher with str:uav name"""
        topic_name = uav_name+"/utm_control"
        uav_state_cmd_pub = rospy.Publisher(topic_name, Int8, queue_size = 10)
        #print(uav_state_cmd_pub)
        return uav_state_cmd_pub

    def send_utm_state_command(self,state_val):
        state_cmd_msg = Int8()
        state_cmd_msg.data = state_val
        self.uav_state_cmd_pub.publish(state_cmd_msg)

    def generate_uav_state(self, uav_name):
        """generates a ros subscriber with str:uav_name"""
        topic_name = uav_name+"/mavros/mode"

    def generate_uav_position_sub(self, uav_name):
        """generates a ros subscriber with str:uav_name"""
        topic_name = uav_name+"/mavros/offset_local_position/pose"
        uav_loc_sub = rospy.Subscriber(topic_name, PoseStamped, self.position_cb)
        return uav_loc_sub

    def position_cb(self,msg):
        x = msg.pose.position.x 
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.coords = [x,y]
        #self.three_cords = [x,y,z]
        #print(self.coords)  
        #return coords



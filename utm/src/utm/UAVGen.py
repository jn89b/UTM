#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from mavros_msgs.msg import State

class UAVComms():
    """uav communication protocol"""
    def __init__(self,name):
        self.name = name 
        self.uav_pos_pub = self.generate_uav_pos_command(self.name)
        self.uav_state_cmd_pub = self.generate_uav_state_command(self.name)

        self.uav_state_sub = self.generate_uav_state_sub(self.name)
        self.mode = None
        self.armed = None
        
        self.uav_loc_sub = self.generate_uav_position_sub(self.name)
        self.coords = [None,None]
        self.three_coords = [None,None,None]
        self.wp_index = 0
        self.wp_list = []

        self.mavros_sub = self.generate_mavros_position_cb(self.name)
        self.mavros_coords = [None, None, None]
        self.state_command = None 
        #self.utm_sub = self.generate_utm_command_sub(self.name)
        

    def get_uav_coords(self):
        """accessor"""
        return self.coords

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
        wp_msg.pose.position.z = wp_coords[2]
        #print(wp_msg)
        self.uav_pos_pub.publish(wp_msg)

    def generate_uav_state_command(self, uav_name):
        """generate a ros publisher with str:uav name"""
        topic_name = uav_name+"/utm_control"
        uav_state_cmd_pub = rospy.Publisher(topic_name, Int8, queue_size = 10)
        #print(uav_state_cmd_pub)
        return uav_state_cmd_pub

    def generate_utm_command(self, uav_name):
        """generate a ros publisher with str:uav name"""
        topic_name = uav_name+"/utm_control"
        uav_state_cmd_pub = rospy.Sub(topic_name, Int8, queue_size = 10)
        #print(uav_state_cmd_pub)
        return uav_state_cmd_pub

    def send_utm_state_command(self,state_val):
        state_cmd_msg = Int8()
        state_cmd_msg.data = state_val
        self.uav_state_cmd_pub.publish(state_cmd_msg)

    def generate_uav_state_sub(self, uav_name):
        """generates a ros subscriber with str:uav_name"""
        topic_name = uav_name+"/mavros/state"
        uav_state_sub = rospy.Subscriber(topic_name, State, self.state_cb)
        return uav_state_sub

    def state_cb(self,msg):
        self.mode = msg.mode
        self.armed = msg.armed
        #print(self.mode, self.armed)

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
        self.three_coords = [x,y,z]
        #print(self.coords)  
        #return coords

    def generate_mavros_position_cb(self, uav_name):
        """generates a ros subscriber with str:uav_name"""
        topic_name = uav_name+"/mavros/local_position/pose"
        mavros_position_sub = rospy.Subscriber(topic_name, PoseStamped, self.mavros_cb)
        return mavros_position_sub
    

    def mavros_cb(self, msg):
        x = msg.pose.position.x 
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.mavros_coords = [x,y,z]

    def utm_cb(self, msg):
        self.state_command = msg.data


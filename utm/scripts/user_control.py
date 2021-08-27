#!/usr/bin env python3

import rospy
import tf
import numpy as np
import threading
import concurrent.futures

from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Bool, Int8

"""
Class precision listens to drone and hears a request from drone to land
if it does request a land -> probably make this a service? allow permission
"""

class UserControl():

    def __init__(self):

        """
        1 = track
        2 = precland
        3 = land
        4 = disarm 
        5 = sendwaypoint
        """
        self.user_cmds_dict = {
            "track": self.track_cmd,
            "precland": self.precland_cmd,  
            "land": self.land_cmd,
            "disarm": self.disarm_cmd,
            "send waypoint": self.waypoint_cmd
        }
        self.cmd = None

        self.user_control_pub = rospy.Publisher("user_control", Int8, queue_size= 10)
        self.pub = rospy.Publisher("precland", Bool, queue_size=10)
        self.sub = rospy.Subscriber("target_found", Bool, self.target_foundcb)
        quad_odom_sub = rospy.Subscriber("mavros/offset_local_position/pose", PoseStamped, self.quad_odom_cb)
        
        self.user_input = Int8()
        
        self.target_found = False
        self.allow_land = Bool()
        self.z = 0.0

    def target_foundcb(self,msg):
        self.target_found = msg.data

    def quad_odom_cb(self,msg):
        z = msg.pose.position.z

    #### COMMAND PROTOCOLS ##############################################
    def track_cmd(self):
        self.user_input.data = 1
        print("im tracking")
        self.user_control_pub.publish(self.user_input)

    #need to make sure that we quad is also stablized and that the error of
    #tag and drone is within tolerance to allow safe landing
    def precland_cmd(self):
        self.user_input.data = 2
        self.user_control_pub.publish(self.user_input)
        self.check_permission

    def check_permission(self):
        self.user_control_pub.publish(self.user)
        if self.target_found == True or self.z < 0.8: #probably need to set this better 
            self.allow_land.data = True
            self.pub.publish(self.allow_land)
        else: 
            self.allow_land.data = False
            self.pub.publish(self.allow_land)

    def land_cmd(self):
        print("I'm landing")

    def disarm_cmd(self):
        print("I'm disarming")

    def waypoint_cmd(self):
        print("Waypoint cmd")

    def listen_for_cmd(self):
        self.listen_cmd = input("listen a command: ")
        print("I heard", self.listen_cmd)
        
        return self.listen_cmd 

    def main(self):

        rate_val = 30
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            try: 
                self.cmd = input("Enter your command: ")
                if self.cmd in self.user_cmds_dict:
                    self.user_cmds_dict.get(self.cmd)()
                else:
                    print("You're wrong")
            except:
                continue 
            
            rate.sleep()

if __name__=='__main__':
    rospy.init_node("user_control", anonymous=True)
    
    usercontrol = UserControl()
    usercontrol.main()
   

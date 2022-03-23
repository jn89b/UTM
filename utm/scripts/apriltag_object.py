#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
from ros_wrapper import utm_airsim_wrapper
from geometry_msgs.msg import PoseStamped

""" 
get true position and subtract from px4 position

"""
class TrueDistance():
    def __init__(self):
        self.px4_pos = [None,None,None]
        sub = rospy.Subscriber('/PX4_0/global_position/pose', PoseStamped, self.px4_cb)
        self.pub = rospy.Publisher('true_tag_dis', PoseStamped, queue_size=10)
    
    def px4_cb(self,msg):
        """get position"""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        self.px4_pos = [x,y,z]
        
    def compute_true_displacement(self, pos2):
        """computes true position displacements between position 1 and position 2"""
        return [pos2[0]-self.px4_pos[0], pos2[1]-self.px4_pos[1], pos2[2]-self.px4_pos[2]]

    def publish_displacement(self, displacement_pos):
        """publishes the displacement"""
        now = rospy.Time.now()
        position_msg = PoseStamped()
        position_msg.header.stamp = now
        position_msg.pose.position.x = displacement_pos[0]
        position_msg.pose.position.y = displacement_pos[1]
        position_msg.pose.position.z = displacement_pos[2]
        self.pub.publish(position_msg)

if __name__=='__main__':
    
    rospy.init_node('apriltag_global_node')
    wsl_ip = os.getenv('WSL_HOST_IP')
    
    rate_val = 20
    rate = rospy.Rate(rate_val)
    
    apriltag_object_name = 'Rover_BP_2'
    airsim_wrapper = utm_airsim_wrapper.AirsimROSWrapper(wsl_ip)
    apriltag_global_pub = airsim_wrapper.generate_global_pub("apriltag_global")
    
    true_dist = TrueDistance()
    
    while not rospy.is_shutdown():
        ned_position = airsim_wrapper.get_object_global_location(apriltag_object_name)
        enu_position, enu_quat = airsim_wrapper.convert_ned_to_enu(ned_position)
        airsim_wrapper.publish_global_position(apriltag_global_pub,'apriltag_frame', 
                                               enu_position,enu_quat)
        
        if true_dist.px4_pos != [None,None,None]:
            displacement_pos = true_dist.compute_true_displacement(enu_position)
            true_dist.publish_displacement(displacement_pos)
            
        rate.sleep()
#!/usr/bin/env python

import roslib
import rospy
import tf

from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped

import roslib
import rospy
import tf

from geometry_msgs.msg import Vector3, Point, PoseStamped, TwistStamped, PointStamped

# MAVROS TF broadcasts the local position frame of the quad wrt to map
"""
Look up transformation or offset of drone  when it spawns
publish the offset as the offset_local_position_pose
"""
class MavrosTF():

    def __init__(self):
        #init tf broacast
        self.br = tf.TransformBroadcaster()
        self.tf_listener_ = tf.TransformListener()
        self.old_target_tf = rospy.get_param("~old_target_tf", "downwards_custom_0_optical")
        #init transformation we will broadcast 
        self.new_tf = rospy.get_param("~quad_tf", "/uav0_wrt_world")
        #source tf we will want to make a transformation of Rnew_tf/source_tf
        self.new_source_tf = rospy.get_param("~world_tf", "world_enu")
        #offset from airsim 
        self.offset_z = rospy.get_param("~offset_z", 0.05)
        # Current drone position (local frame)
        self.drone_pos = [0,0,0,0,0,0,0]
        self.tf_listener_ = tf.TransformListener()
        self.pub = rospy.Publisher("mavros/offset_local_position/pose",PoseStamped, queue_size=10)

    #convert transformation R mavros/px4 odom
    def broadcast_drone_tf(self):
        try:
            now = rospy.Time.now()
            self.tf_listener_.waitForTransform(self.new_source_tf, self.old_target_tf, rospy.Time(0), rospy.Duration(15.0))
            (trans,rot) = self.tf_listener_.lookupTransform(self.new_source_tf,self.old_target_tf,rospy.Time(0))
            x = trans[0]
            y = trans[1]
            z = trans[2] + 0.5
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
            rot = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
            self.br.sendTransform((x,y,z),(rot[0],rot[1],rot[2],rot[3]),now,self.new_tf, self.new_source_tf)
            #publish posestamped
            posestamped = PoseStamped()
            posestamped.header.frame_id = self.new_tf
            posestamped.pose.position.x = x
            posestamped.pose.position.y = y
            posestamped.pose.position.z = z

            posestamped.pose.orientation.x = rot[0]
            posestamped.pose.orientation.y = rot[1]
            posestamped.pose.orientation.z = rot[2]
            posestamped.pose.orientation.w = rot[3]

            self.pub.publish(posestamped)
        except (tf.LookupException, tf.ConnectivityException):
            return 

    def main(self):
        rate = rospy.Rate(10)
        self.tf_listener_.waitForTransform(self.new_source_tf, self.old_target_tf, rospy.Time(0), rospy.Duration(2.5))
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.tf_listener_.waitForTransform(self.new_source_tf, self.old_target_tf, rospy.Time(0), rospy.Duration())
                (trans,rot) = self.tf_listener_.lookupTransform(self.new_source_tf,self.old_target_tf,rospy.Time(0))
                x = trans[0]
                y = trans[1]
                z = trans[2] + 0.5
                #(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
                rot = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
                self.br.sendTransform((x,y,z),(rot[0],rot[1],rot[2],rot[3]),now,self.new_tf, self.new_source_tf)
                #publish posestamped
                posestamped = PoseStamped()
                posestamped.header.frame_id = self.new_tf
                posestamped.pose.position.x = x
                posestamped.pose.position.y = y
                posestamped.pose.position.z = z

                posestamped.pose.orientation.x = rot[0]
                posestamped.pose.orientation.y = rot[1]
                posestamped.pose.orientation.z = rot[2]
                posestamped.pose.orientation.w = rot[3]

                self.pub.publish(posestamped)
                    
            except (tf.LookupException, tf.ConnectivityException):
                continue
                #rate.sleep()
        rate.sleep()
            
if __name__ == '__main__':
    # Initiate node
    rospy.init_node("mavros_tf", anonymous=True)
    mavrostf = MavrosTF()
    mavrostf.main()


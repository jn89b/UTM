#!/usr/bin/env python

from re import S
import random
from operator import add,sub
import numpy as np 
import rospy 
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped

from apriltag_ros.msg import AprilTagDetectionArray

class AprilTagPositionPub():
    def __init__(self):
        #apriltag id 
        self.tag_id_ = rospy.get_param("~tag_id",0)
        self.new_tf = rospy.get_param("~rtag_drone", "tag_wrt_uav0")       
        self.drone_frame_id_ = rospy.get_param("~quad_tf", "/uav0_wrt_world")
        self.tag_frame_id_ = rospy.get_param("~tag_frame_id", "/tag_0")
        #self.tags_topic_ = rospy.get_param('~tags_topic', '/tag_detections')
        self.setpoint_topic_ = rospy.get_param('~setpoint_topic', '/setpoint/relative_pos')
        self.tf_listener_ = tf.TransformListener()
        #broadcast tf transform
        self.br = tf.TransformBroadcaster()
        #init transformation we will broadcast 
 
        # Desired altitude above the tag, meters
        self.alt_from_tag_ = rospy.get_param('~alt_from_tag', 1.0)
        # Sanity check. alt_from_tag_ should be non-negative. Otherwise tag will not be seen!
        if self.alt_from_tag_ < 0.0 :
            rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0


        self.camera_z_offset = 0.5

        self.apriltag_position = [None,None,None]
        self.apriltag_orientation = [None,None,None,None]
        
        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=20)

        # Relative pose publisher as PoseStamped
        self.pose_pub_ = rospy.Publisher('tag/pose', PoseStamped, queue_size=20)

        #boolean statement 
        self.target_pub = rospy.Publisher("target_found", Bool, queue_size=20)
        # Subscriber to Kalman filter estimate
        #rospy.Subscriber("kf/estimate", PoseWithCovarianceStamped, self.kfCallback)

        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.tagsCallback)

        self.valid = False
                
    def check_wrap(self, val):
        """check for angle wrapping"""
    
    # tags callback
    def tagsCallback(self, msg):
        valid = False
        trans = []
        scale_factor = 1.0
        camera_offset_z = 0.5  
        if len(msg.detections) > 0: # make sure detection is valid
            overall_pose = msg.detections[0].pose.pose.pose
            x = overall_pose.position.x/scale_factor
            y = overall_pose.position.y/scale_factor
            z = overall_pose.position.z/scale_factor            
            qx = overall_pose.orientation.x
            qy = overall_pose.orientation.y
            qz = overall_pose.orientation.z
            qw = overall_pose.orientation.w
            
            self.apriltag_position = [x, y, z]
            
            self.apriltag_orientation = [qx, qy, qz, qw]
            

            # (trans,rot) = self.tf_listener_.lookupTransform(self.tag_frame_id_, self.drone_frame_id_, rospy.Time(0))
            # target_found = Bool()
            # target_found.data = True
            # self.target_pub.publish(target_found)
            self.valid = True
        else:
            self.valid = False
            # target_found = Bool()
            # target_found.data = False
            # self.target_pub.publish(target_found) 
            # #rospy.logwarn("No valid TF for the required tag %s", self.tag_id_)
            # return

        # if valid: # Publish relative setpoint
        #     now = rospy.Time.now()
        #     # pose_msg = PoseStamped()
        #     # pose_msg.header.frame_id = self.new_tf
        #     # pose_msg.header.stamp = rospy.Time.now()
        #     # pose_msg.pose.position.x = trans[0]/scale_factor 
        #     # pose_msg.pose.position.y = trans[1]/scale_factor
        #     # pose_msg.pose.position.z = trans[2] - camera_offset_z
        #     # pose_msg.pose.orientation.x = rot[0]
        #     # pose_msg.pose.orientation.y = rot[1]
        #     # pose_msg.pose.orientation.z = rot[2]
        #     # pose_msg.pose.orientation.w = rot[3]
            
        #     pose_msg = PoseStamped()
        #     pose_msg.header.frame_id = self.new_tf
        #     pose_msg.header.stamp = rospy.Time.now()
        #     pose_msg.pose.position.x = self.apriltag_position[0] 
        #     pose_msg.pose.position.y = self.apriltag_position[1]
        #     pose_msg.pose.position.z = self.apriltag_position[2] - camera_offset_z
        #     #pose_msg.pose.orientation = self.apriltag_orientation
            
        #     # pose_msg.pose.orientation.x = self.apriltag_position[0]
        #     # pose_msg.pose.orientation.y = rot[1]
        #     # pose_msg.pose.orientation.z = rot[2]
        #     # pose_msg.pose.orientation.w = rot[3]
        #     self.pose_pub_.publish(pose_msg)
        #     #sending transform rtagwrtdrone Rtag/drone
        #     #self.br.sendTransform((trans[0]/scale_factor,trans[1]/scale_factor, trans[2]- camera_offset_z),(rot[0],rot[1],rot[2],rot[3]),now,self.new_tf, self.drone_frame_id_)
            
        # else:
        #     pass
        
    def publish_apriltag(self, trans, rot):
        """publish apriltag position"""
        pose_msg = PoseStamped()
        
        #adding noise
        ops = (add,sub)
        op = random.choice(ops)
        
        noise_x = random.uniform(0,1) *0.25
        noise_y = random.uniform(0,1) *0.25
        
        px = op(trans[1], noise_x)
        py = op(trans[0], noise_y)
        
        pose_msg.header.frame_id = self.new_tf
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = px #trans[1]     
        pose_msg.pose.position.y = -py #-trans[0] 
        pose_msg.pose.position.z = trans[2] - self.camera_z_offset
        pose_msg.pose.orientation.x = rot[1]
        pose_msg.pose.orientation.y = -rot[0]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]
        self.pose_pub_.publish(pose_msg)
        
    def main(self, rate_val):
        """begin publishing the apriltag position wrt drone and broadcast transform"""
        
        rate_val = 20
        rate = rospy.Rate(rate_val)
        

        while not rospy.is_shutdown():
            self.tf_listener_.waitForTransform(
                self.drone_frame_id_, self.tag_frame_id_,  rospy.Time(),rospy.Duration(15.0))
            
            try:
                if self.valid == True:                    
                    # self.tf_listener_.waitForTransform(
                    #      self.drone_frame_id_, self.tag_frame_id_, rospy.Time(), rospy.Duration(3.0))
                    
                    (trans,rot) = self.tf_listener_.lookupTransform(
                         self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
                    
                    self.publish_apriltag(trans,rot)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
                
            rate.sleep()
            
                
if __name__ == '__main__':
    rospy.init_node('apriltag_position_pub', anonymous=True)

    rate_val = 20
    rate = rospy.Rate(rate_val)
    sp_o = AprilTagPositionPub()
    #(trans,rot) = sp_o.tf_listener_.lookupTransform(sp_o.drone_frame_id_, sp_o.tag_frame_id_, rospy.Time(0))
    #while not rospy.is_shutdown():
    #sp_o = AprilTagPositionPub()
    sp_o.main(rate_val)
    #rate.sleep()
        
    #rospy.spin()

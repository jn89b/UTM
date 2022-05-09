#!/usr/bin/env python

from re import S
import random
from operator import add,sub
import numpy as np 
import rospy 
import tf
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion

from apriltag_ros.msg import AprilTagDetectionArray
from nav_msgs.msg import Odometry
import math

class AprilTagPositionPub():
    def __init__(self):
        #apriltag id 
        self.tag_id_ = rospy.get_param("~tag_id",0)
        self.new_tf = rospy.get_param("~rtag_drone", "tag_wrt_uav0")       
        self.drone_frame_id_ = rospy.get_param("~quad_tf", "/PX4_0_wrap")
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
            #rospy.logerr("Desired altitude above tag is negative. It should be positive. Defaulting to 1.0 meter")
            self.alt_from_tag_ = 1.0
            
        self.camera_z_offset = 0.5
        self.apriltag_position = [None,None,None]
        self.apriltag_orientation = [None,None,None,None]
        
        # Relative setpoint publisher
        self.setpoint_pub_ = rospy.Publisher(self.setpoint_topic_, Point, queue_size=20)

        # Relative pose publisher as PoseStamped
        
        self.pose_pub_ = rospy.Publisher('displace/tag/pose', PoseStamped, queue_size=20)
        self.target_pub = rospy.Publisher("target_found", Bool, queue_size=20)

        self.sub = rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, self.current_state)
        self.z_sub = rospy.Subscriber("/PX4_0/global_position/pose", PoseStamped, self.z_cb)
        rospy.Subscriber('uav0/tag_detections', AprilTagDetectionArray, self.tagsCallback)

        #boolean statement 
        self.pitch_rad = 0.0 
        self.roll_rad = 0.0 
        self.pitch_deg = 0.0 
        self.roll_deg = 0.0 
        
        self.dx = 0.0 
        self.dy = 0.0
        self.z = 0.0 
        self.valid = False
                
    def current_state(self, msg):
        """update current estimates"""        
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.pitch_rad = pitch   
        self.roll_rad = roll
        self.pitch_deg = pitch * 180/math.pi
        self.roll_deg = roll * 180/math.pi
        
    def z_cb(self,msg):
        """get z ground truth"""
        self.z = msg.pose.position.z

    def compute_offset(self):
        """compute offset distortion from drone attitude"""
        self.dx = self.z * np.tan(self.pitch_rad)
        self.dy = self.z * np.tan(self.roll_rad)
        print(self.dx, self.dy)
    
    def tagsCallback(self, msg):
        trans = []
        scale_factor = 1.0
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
            
    def publish_apriltag(self, trans, rot):
        """publish apriltag position"""
        pose_msg = PoseStamped()
        
        #added uniform noise
        ops = (add,sub)
        op = random.choice(ops)
        
        noise_x = random.uniform(0,1) *0.1
        noise_y = random.uniform(0,1) *0.1
        
        px = op(trans[1], noise_x)
        py = op(trans[0], noise_y)
        
        pose_msg.header.frame_id = self.new_tf
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = px - self.dx#trans[1] SIGN FLIP FML   
        pose_msg.pose.position.y = -py + self.dy #-trans[0] SIGN FLIP FML
        pose_msg.pose.position.z = trans[2] - self.camera_z_offset
        pose_msg.pose.orientation.x = rot[1]
        pose_msg.pose.orientation.y = -rot[0]
        pose_msg.pose.orientation.z = rot[2]
        pose_msg.pose.orientation.w = rot[3]
        self.pose_pub_.publish(pose_msg)
        
    def main(self, rate_val):
        """begin publishing the apriltag position wrt drone and broadcast transform"""
        
        #rate_val = 30
        rate = rospy.Rate(rate_val)
        
        while not rospy.is_shutdown():
            try:
                if self.valid == True:                    
                    # self.tf_listener_.waitForTransform(
                    #      self.drone_frame_id_, self.tag_frame_id_, rospy.Time(), rospy.Duration(3.0))
                    (trans,rot) = self.tf_listener_.lookupTransform(
                         self.drone_frame_id_, self.tag_frame_id_, rospy.Time(0))
                    self.compute_offset()
                    self.publish_apriltag(trans,rot)

                #rate.sleep()
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
                
            rate.sleep()
            
                
if __name__ == '__main__':
    rospy.init_node('displace_pose', anonymous=True)

    rate_val = 30
    rate = rospy.Rate(rate_val)
    sp_o = AprilTagPositionPub()
    #(trans,rot) = sp_o.tf_listener_.lookupTransform(sp_o.drone_frame_id_, sp_o.tag_frame_id_, rospy.Time(0))
    #while not rospy.is_shutdown():
    #sp_o = AprilTagPositionPub()
    
    sp_o.main(rate_val)
    #rate.sleep()
        
    #rospy.spin()

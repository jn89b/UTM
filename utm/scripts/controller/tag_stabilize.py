"""
Stabilize fiducial tag target

- Find attitude of quad in roll and pitch
- Subscribe to position of tag
- if tag position is "good" that is attitude estimates between -3 to 3 degrees:
    store this position as a reference
    set this as some relative truth estimate
- keep looping
"""
from airsim.types import Pose
import rospy
import math
# import time
# import csv
# import os
# import datetime

from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

class WebSling():
    
    """_summary_
    Spiderman likes to swing from one stable point to another point    
    """
    
    def __init__(self):
        uav_name = "uav0"
        att_topic =  uav_name+"/mavros/odometry/in"
        tag_topic_filtered = uav_name+"/tag/pose"
        websling_topic_name = "websling"
        self.sub = rospy.Subscriber(att_topic, Odometry, self.current_state)
        self.tag_sub = rospy.Subscriber(tag_topic_filtered, PoseStamped, self.kftag_cb) 
        self.web_pub = rospy.Publisher(websling_topic_name, PoseStamped, queue_size=10)

        self.pitch_rad = 0.0 
        self.roll_rad = 0.0 
        self.pitch_deg = 0.0 
        self.roll_deg = 0.0 
        
        self.kftag = [0.0, 0.0, 0.0]
        self.sling_point = [0.0, 0.0,0.0]
        self.angle_tol = 10.0  #degrees
        
    def kftag_cb(self,msg):
        self.kftag[0] = msg.pose.position.x
        self.kftag[1] = msg.pose.position.y
        self.kftag[2] = msg.pose.position.z

    def current_state(self, msg):
        """update current estimates"""        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        self.pitch_rad = pitch   
        self.roll_rad = roll
        self.pitch_deg = pitch * 180/math.pi
        self.roll_deg = roll * 180/math.pi

    def is_stabilize(self):
        """check if attitude of drone is stable"""
        if (abs(self.pitch_deg) < self.angle_tol) and (abs(self.roll_deg) < self.angle_tol):
            return True
        else: 
            return False  

    def set_sling_point(self):
        """set point to traverse through next"""
        self.sling_point[0] = self.kftag[0]
        self.sling_point[1] = self.kftag[1]
        self.sling_point[2] = self.kftag[2]
    
    def publish_sling_point(self):
        """publish the sling point message as PoseStamped"""
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.sling_point[0]
        pose_msg.pose.position.y = self.sling_point[1]
        pose_msg.pose.position.z = self.sling_point[2]
        self.web_pub.publish(pose_msg)
        
    def main(self):
        """main implementation to set anchor points"""
        # set new reference points if stablr
        pose_msg = PoseStamped()
        if (abs(self.pitch_deg) < self.angle_tol):
            self.sling_point[0] = self.kftag[0]
            #print("less", self.pitch_deg)
        else:
            print("not stable", self.pitch_deg)
        
        if (abs(self.roll_deg) < self.angle_tol):
            self.sling_point[1] = self.kftag[1]
            #print("less", self.roll_deg)
        else:
            print("not stable", self.roll_deg)
        
        #rospy.sleep(1E-3)
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.sling_point[0]
        pose_msg.pose.position.y = self.sling_point[1]
        pose_msg.pose.position.z = self.sling_point[2]
        self.web_pub.publish(pose_msg)
        
        #self.publish_sling_point()

if __name__=='__main__':
    rospy.init_node('web_sling')
    
    rate_val = 30
    rate = rospy.Rate(rate_val) 
    websling = WebSling()
    print("Spider-Man Spider-Man does whatever a Spider Can")
    while not rospy.is_shutdown():
        websling.main()
    rate.sleep()
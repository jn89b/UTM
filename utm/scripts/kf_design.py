#!/usr/bin/env python

import rospy 
import tf


import numpy as np
from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray

"""
class kalman filter subscribes to lateral position of tag and makes an estimate on position of 
of tag wrt to quad based on the camera sensor also makes an estimate on how fast tag is moving 
"""
class KalmanFilter():
    def __init__(self, F = None, B = None, H = None, Q = None, R = None, P = None, x0 = None):

        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1]

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        
        #desired states
        self.z = [0] * len(self.H)

        self.pz = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0
        
        self.z_height = None
        #self.quat_list = [None,None,None,None]

        #this is the apriltag position subscriber
        rospy.Subscriber("/uav0/tag/pose", PoseStamped, self.tagpose_cb)
        
        self.kf_pub = rospy.Publisher("uav0/kf_tag/pose", PoseStamped, queue_size=20)        
        self.P_pub = rospy.Publisher("uav0/P_covar/pose", PoseStamped, queue_size=20)
        
        self.vision_pub = rospy.Publisher("uav0/mavros/vision_pose/pose", PoseStamped, queue_size=20)
        self.kf_vel_pub = rospy.Publisher("uav0/kf_tag/vel", TwistStamped, queue_size=20)
        self.uav_height_sub = rospy.Subscriber("/uav0/mavros/odometry/in", Odometry, self.height_cb)
        
    def update_R(self):
        """update R based on height param"""
        if self.z_height != None:
            R_factor = (0.00232*self.z_height + 0.0295)**2
            # rospy.logwarn("R set at %s", self.R)
        else:
            R_factor = (0.1)**2
            
        self.R = np.array([[R_factor, 0], [0, R_factor]]) #measurement noise for kalman filter

    def predict(self, u = 0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.publish_kf_est() #publish the kf estimates for position and vel of tag
        self.publish_P()
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self):
        y = self.z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y) #udpate state matrix
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
        	(I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

    def tagpose_cb(self,msg): 
        """get z readings """
        px = msg.pose.position.x 
        py = msg.pose.position.y        
        self.pz = msg.pose.position.z
        
        self.qx = msg.pose.orientation.x
        self.qy = msg.pose.orientation.y
        self.qz = msg.pose.orientation.z
        self.qw = msg.pose.orientation.w
        
        self.z = np.array([[px,py]]).T
        #return self.z 

    def height_cb(self,msg): 
        """height callback"""
        self.z_height = msg.pose.pose.position.z
        #return self.z 
        
    def publish_P(self):
        """publishes error covariance estimate,P"""
        now = rospy.Time.now()
        # print(self.P)
        # print(self.P[1,1])
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "P_error"
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = self.P[0,0] 
        pose_msg.pose.position.y = self.P[1,1]        
        self.P_pub.publish(pose_msg)

    def publish_kf_est(self):
        now = rospy.Time.now()
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "ekf_tag"
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = self.x[0,0] 
        pose_msg.pose.position.y = self.x[1,0]
        pose_msg.pose.position.z = self.pz
        pose_msg.pose.orientation.x = self.qx
        pose_msg.pose.orientation.y = self.qy
        pose_msg.pose.orientation.z = self.qz
        pose_msg.pose.orientation.w = self.qw
        
        self.kf_pub.publish(pose_msg)
        self.vision_pub.publish(pose_msg)
        
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = "ekf_tag_vel"
        vel_msg.header.stamp = now
        vel_msg.twist.linear.x = self.x[2,0]
        vel_msg.twist.linear.y = self.x[3,0]
        self.kf_vel_pub.publish(vel_msg)

if __name__ == "__main__":
    rospy.init_node("ekf_tag", anonymous=True)
    print("starting")
    Q_param = rospy.get_param("~Q_param", "0.001")
    Q_fact =  float(Q_param) # process noise covariance constant
    rate_val = 15
    #init vals
    dt = 1/rate_val   
     
    ####### CONSTANT ACCELERATION MODEL##########
    
    #This array is for constant acceleartion so size 6
    x_1 = [1, 0.0, dt, 0.0, 1/2.0*dt**2, 0.0] #px  
    x_2 = [0.0, 1, 0.0, dt, 0.0,  1/2.0*dt**2] #py 
    x_3 = [0.0, 0.0 , 1, 0.0, dt, 0.0] #vx
    x_4 = [0.0, 0.0, 0.0, 1, 0.0, dt]  #vy
    x_5 = [0.0, 0.0, 0.0, 0.0, 1, 0.0] #ax
    x_6 = [0.0, 0.0, 0.0, 0.0, 0.0, 1] #ay

    F = np.array([x_1, x_2, x_3, x_4, x_5, x_6]) #feeding in x values in array
    # print(F.shape)

    h_1 = [1, 0.0, 0.0, 0.0, 0.0, 0.0] #measurement of px
    h_2 = [0.0, 1, 0.0, 0.0, 0.0, 0.0] #measurement of py

    H = np.array([h_1, h_2])
    #print(H.shape)

    Q = np.array([[Q_fact, 0, 0, 0, 0, 0], 
                [0, Q_fact, 0, 0, 0, 0], 
                [0, 0, Q_fact, 0, 0, 0], 
                [0, 0 , 0, Q_fact, 0, 0],
                [0, 0 , 0, 0, Q_fact, 0],
                [0, 0 , 0, 0, 0, Q_fact]])
    
    ##################################################

    ############ CONSTANT VELOCITY MODEL###############

    # #This model is for constant velocity size 4x4
    # x_1 = [1, 0, dt, 0]
    # x_2 = [0, 1, 0, dt]
    # x_3 = [0, 0, 1, 0]
    # x_4 = [0, 0, 0, 1]

    # F = np.array([x_1, x_2, x_3, x_4])

    # h_1 = [1, 0, 0, 0]
    # h_2 = [0, 1, 0, 0]

    # H = np.array([h_1, h_2])

    # Q_fact = 1E-4 #process noise variance
    # Q = np.array([[Q_fact, 0, 0, 0],
    #             [0, Q_fact, 0, 0],
    #             [0, 0, Q_fact, 0],
    #             [0, 0, 0, Q_fact]])

    # ##################################################

    # ##### NOISE FACTOR AND INPUT TO KALMAN FILTER
    R_factor = (0.1)**2 # measurement of camera saying .3m off
    R = np.array([[R_factor, 0], [0, R_factor]]) #measurement noise for kalman filter

    kf = KalmanFilter(F = F, H = H, Q = Q, R = R) #import matrices into class

    rate = rospy.Rate(rate_val)

    while not rospy.is_shutdown():
        kf.update_R()
        kf.predict()
        kf.update()
        rate.sleep()

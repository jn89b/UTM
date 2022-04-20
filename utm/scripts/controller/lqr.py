#!/usr/bin/env python

from this import d
import rospy 
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import scipy
from scipy.integrate import odeint
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64

from mavros_msgs.msg import AttitudeTarget
from utm.msg import LQRGain

""" 
LQR control for position tracking and referencing 
of fiducial tags

Drone in sim is AR 2.0 
weight is 13.4 oz 
Ix = 8.1 * 1e-3
Iy = 8.1 * 1e-3
Iz = 14.2 * 1e-3

LQR procedures
    #get act state done by callbacks already
    
    #get desired state done by callbacks already
    
    #compute state error 
    
    #do lqr and get my gains

    #update state space model done by callbacks already
    
"""
Ix = 8.1 * 1E-3
Iy = 8.1 * 1E-3
g = 9.81 #m^2/s
m = 0.37 #kg


class LQR():
    def __init__(self, A = None, B = None, Q = None, R = None, x0 = None,
                 rate_val = None):     
        if(A is None or B is None):
            raise ValueError("Set proper system dynamics.")

        self.n = A.shape[0]
        self.m = B.shape[1]
        
        self.A = A
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) #if Q is None else Q
        
        #self.R = np.eye(self.n) if R is None else R
        self.R = np.diag([20])
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        
        
        #gains
        self.K = []
        
        #desired states
        self.z = [0] * len(Q)
        self.error = [0] * len(Q)
        self.lqr_output = [0] * len(Q)

        #rate values
        self.rate_val = 50 if rate_val is None else rate_val
        self.dt = 1/self.rate_val
        
        #quad position callback
        self.quad_sub = rospy.Subscriber("uav0/mavros/odometry/in",
                                         Odometry,
                                         self.current_state)
        
        self.track_sub = rospy.Subscriber("uav0/mavros/vision_pose/pose", 
                                                 PoseStamped,
                                                 self.desired_state)
        
        self.k_pub = rospy.Publisher("K_gain", LQRGain, queue_size=5)
                
    def current_state(self, msg):
        """update position estimate """
        px = msg.pose.pose.position.x 
        #py = msg.pose.position.y
        vel_x = msg.twist.twist.linear.x
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        pitch_rate = pitch - self.x[2]/self.dt
         
        self.x[0] = px
        self.x[1] = vel_x
        self.x[2] = pitch
        self.x[3] = pitch_rate
                
    def desired_state(self, msg):
        """get desired position from current position"""
        desired_x = msg.pose.position.x # - self.x[0]
        #des_y = msg.pose.position.y
        #des_vel_x = (desired_x - self.z[0])/self.dt 
        self.z[0] = desired_x
        self.z[1] = 0.0
        self.z[2] = 0.0
        self.z[3] = 0.0

    def compute_error(self):
        """compute error of state"""
        self.error[0] = self.z[0] 
        self.error[1] = self.z[1] - self.x[1]
        self.error[2] = self.z[2] - self.x[2]
        self.error[3] = self.z[3] - self.x[3]
        
        print("ERROR", self.error[0])
        
    def lqr(self, A, B, Q, R):
        """Solve the continuous time lqr controller.
        dx/dt = A x + B u
        cost = integral x.T*Q*x + u.T*R*u
        """
        # http://www.mwm.im/lqr-controllers-with-python/
        # ref Bertsekas, p.151

        # first, try to solve the ricatti equation
        X = np.matrix(scipy.linalg.solve_continuous_are(A, B, Q, R))

        # compute the LQR gain
        K = np.matrix(scipy.linalg.inv(R) * (B.T * X))

        eigVals, eigVecs = scipy.linalg.eig(A - B * K)
        return np.asarray(K), np.asarray(X), np.asarray(eigVals)

    def compute_K(self):
        """get gains from LQR"""
        self.Q[0,0] = 0.45
        K, _, _ = self.lqr(self.A, self.B, self.Q, self.R)
        self.K = K
        
    def update_state(self): 
        """update state space"""
        self.lqr_output = self.B * self.u
        self.x = np.dot(self.A, self.x)  + self.lqr_output
        
    def get_u(self):
        """compute controller input""" 
        self.u = np.dot(self.K, self.error)[0]
        max_pitch = 4.5
        if abs(self.u)>= max_pitch:
            if self.u > 0:
                self.u = max_pitch
            else:
                self.u = -max_pitch

    def publish_gains(self):
        """publish K gains"""
        if self.K[0,0]!= None:
            self.k_pub.publish(float(self.K[0,0]))
        else:
            self.k_pub.publish(0.0)

    def publish_input(self):
        """publish body rate commands"""
        gains = LQRGain()
        if abs(self.error[0]) <= 3.0:
            self.k_pub.publish([0.0])
        else:
            gains.data = [self.u]

            self.k_pub.publish(gains)
                
    def main(self):
        """update values to LQR"""
        self.compute_error()
        self.compute_K()
        self.get_u()
        self.publish_input()
        self.update_state()
        
if __name__ == "__main__":
    
    rospy.init_node("lqr_controller", anonymous=False)
    rate_val = 15

    ############ Set up X and Y #####################
    # X-subsystem
    # The state variables are x, dot_x, pitch, dot_pitch
    Ax = np.array(
        [[0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]])
    
    Bx = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / Ix]])
    
    # Y-subsystem
    # The state variables are y, dot_y, roll, dot_roll
    Ay = np.array(
        [[0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, -g, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0, 0.0]])
    
    By = np.array(
        [[0.0],
        [0.0],
        [0.0],
        [1 / Iy]])
    
    ## Q penalty
    Q_fact =  1.0 #penalizes performance rating 
    Q = np.array([[Q_fact, 0, 0], 
                [0, Q_fact, 0, 0], 
                [0, 0, Q_fact/2, 0], 
                [0, 0 , 0, Q_fact/2]])
    
    
    ## R penalty for input
    R = np.array([[Q_fact, 0, 0], 
                [0, Q_fact, 0, 0], 
                [0, 0, Q_fact/2, 0], 
                [0, 0 , 0, Q_fact/2]])
    
    lqr = LQR(A = Ax, B = Bx, Q = Q, R = R, x0 = None,
                 rate_val = rate_val) #import matrices into class

    rate = rospy.Rate(rate_val)
    
    while not rospy.is_shutdown():
        lqr.main()
        rate.sleep()
    
    
    
    
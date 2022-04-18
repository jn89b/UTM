#!/usr/bin/env python

from this import d
import rospy 
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import scipy
from scipy.integrate import odeint

""" 
LQR control for position tracking and referencing 
of fiducial tags

Drone in sim is AR 2.0 
weight is 13.4 oz 
Ix = 8.1 * 1e-3
Iy = 8.1 * 1e-3
Iz = 14.2 * 1e-3
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
        self.R = np.diag([1., ])
        self.x = np.zeros((self.n, 1)) if x0 is None else x0
        #gains
        self.K = []
        
        #desired states
        self.z = [0] * len(Q)
        
        self.rate_val = 50 if rate_val is None else rate_val
        self.dt = 1/self.rate_val
        
        #quad position callback
        self.quad_sub = rospy.Subscriber("uav0/mavros/setpoint_position/local",
                                         PoseStamped,
                                         self.current_state)
        
        self.track_sub = rospy.Subscriber("uav0/mavros/vision_pose/pose", 
                                                 PoseStamped,
                                                 self.desired_state)
        
    def current_state(self, msg):
        """update position estimate """
        px = msg.pose.position.x 
        #py = msg.pose.position.y
        vel_x = (px - self.z[0] )/self.dt
        
        self.x = np.array([[px,vel_x,0,0]]).T 
        
    def desired_state(self, msg):
        """get desired position from current position"""
        des_px = msg.pose.position.x + self.x[0]
        #des_y = msg.pose.position.y
        des_vel_x = (des_px - self.z[0] )/self.dt + self.x[1]
        
        self.z = np.array([[des_px, des_vel_x, 0, 0]]).T
   
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
        self.Q[0,0] = 1.25
        K, _, _ = self.lqr(self.A, self.B, self.Q, self.R)
        self.K = K
            
    def main(self):
        """update values to LQR"""
        #get act state done by callbacks already
        
        #get desired state done by callbacks already
        
        #compute state error 
        
        #get input, B, subscribe to velocity commands? 

        #do lqr
        self.compute_K()
        print(self.K)

        #update state space model 

    
if __name__ == "__main__":
    
    rospy.init_node("lqr_controller", anonymous=True)
    rate_val = 50

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
    Q_fact = 1 #penalizes performance rating 
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
    
    
    
    
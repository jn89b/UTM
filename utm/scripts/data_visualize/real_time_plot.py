#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Plot:
    - Time
    - Fiducial Read
    - Kalman Fiducial Predicted
    - True Position:
        - get true position by:
        - global position of apriltag
        - global position of drone 
        - difference in position -> convert to ENU
            - this is the true position 
    
"""
import matplotlib.pyplot as plt
import rospy
import tf
#from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import quaternion_matrix
import numpy as np
from matplotlib.animation import FuncAnimation


class Visualiser:
    def __init__(self):
        self.kf_x = 0.0
        self.kf_y = 0.0
        self.true_x = 0.0
        self.true_y = 0.0
        
        self.fig, (self.ax1,self.ax2) = plt.subplots(2,1)
        # self.ln, = plt.plot([], [])
        self.colors = ['orange','blue','cyan']
        # self.colors2 = ['green', 'red']
        
        self.labels = ['vision_x', 'kf_est_x','true_position_x']
        self.labels2 = ['vision_y', 'kf_est_y', 'true_position_y']
        
        self.lines = [self.ax1.plot([], [])[0] for _ in range(3)]
        self.lines2 = [self.ax2.plot([], [])[0] for _ in range(3)]
        
        #print("lines are", self.lines, self.lines2)
        
        for i, line in enumerate(self.lines):
            line._color = self.colors[i]
            line.set_label(self.labels[i])
        
        for i, line in enumerate(self.lines2):
            line._color = self.colors[i]
            line.set_label(self.labels2[i])
        
        self.x_vision_list, self.y_vision_list, self.time = [] , [], []
        self.kf_x_list, self.kf_y_list = [], []
        self.true_x_list, self.true_y_list = [], []
        
        self.repeat_length = 100
        
        sub1 = rospy.Subscriber('uav0/tag/pose', PoseStamped, self.tag_cb)
        sub2 = rospy.Subscriber('uav0/kf_tag/pose', PoseStamped, self.kf_cb)
        sub3 = rospy.Subscriber('true_tag_dis', PoseStamped, self.true_cb)

        self.ax1.legend(handles = self.lines)
        self.ax2.legend(handles = self.lines2)
        
    
    def plot_init(self):
        for ax in [self.ax1,self.ax2]:
            ax.set_xlim(left=0, right=self.repeat_length)
            ax.set_ylim([-2,2])
        return self.lines, self.lines2

    def true_cb(self,msg):
        self.true_x = msg.pose.position.x
        self.true_y = msg.pose.position.y 

    def kf_cb(self,msg):        
        self.kf_x = msg.pose.position.x
        self.kf_y = msg.pose.position.y 

    def tag_cb(self, msg):
        time = rospy.get_time() - time_zero
        x_vision = msg.pose.position.x
        y_vision = msg.pose.position.y
                
        time_index = len(self.time)
        self.time.append(time_index+1)  
        
        self.x_vision_list.append(x_vision)
        self.y_vision_list.append(y_vision)
        
        self.kf_x_list.append(self.kf_x)
        self.kf_y_list.append(self.kf_y)
        
        self.true_x_list.append(self.true_x)
        self.true_y_list.append(self.true_y)


        for ax in [self.ax1,self.ax2]:
            if time_index > self.repeat_length:
                ax.set_xlim(time_index-self.repeat_length, time_index)
            else:
                ax.set_xlim(0, self.repeat_length)


    def update_plot(self, frame):
        self.lines[0].set_data(self.time, self.x_vision_list)
        self.lines[0].set_label(self.labels[0])        
        
        self.lines[1].set_data(self.time, self.kf_x_list)
        self.lines[1].set_label(self.labels[1])
        
        self.lines[2].set_data(self.time, self.true_x_list)
        self.lines[2].set_label(self.labels[2])
            
        self.lines2[0].set_data(self.time, self.y_vision_list)
        self.lines2[0].set_label(self.labels2[0])
    
        self.lines2[1].set_data(self.time, self.kf_y_list)
        self.lines2[1].set_label(self.labels2[1])
        
        self.lines2[2].set_data(self.time, self.true_y_list)
        self.lines2[2].set_label(self.labels2[2])
        
        return self.lines, self.lines2

rospy.init_node('visualize_tracking')
plt.close("all")
rate = rospy.Rate(20)
time_zero = rospy.get_time()
vis = Visualiser()

while not rospy.is_shutdown():
    ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, 
                        frames=10,blit=False)
    plt.show(block=True)
    rate.sleep()
    
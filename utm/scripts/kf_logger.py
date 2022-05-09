#!/usr/bin/env python
# -*- coding: utf-8 -*- 

from utm import config

from ipaddress import ip_address
import pandas as pd
import rospy

import time
import csv
import os
import datetime
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class OdomLoc():
	def __init__(self,sub_topic):
		self.x = None
		self.y = None
		self.z = None
		self.coords = [None,None,None]
		self.sub = rospy.Subscriber(sub_topic, PoseStamped, self.odom_cb) 
		self.pitch = None
		self.roll = None
		self.yaw = None

	def odom_cb(self,msg):
		self.x = msg.pose.position.x
		self.y = msg.pose.position.y
		self.z = msg.pose.position.z
		self.coords = [self.x,self.y,self.z]
		q = msg.pose.orientation
		explicit_quat = [q.x, q.y, q.z, q.w]
		(roll, pitch, yaw) = euler_from_quaternion(explicit_quat)
		self.pitch = pitch
		self.roll = roll
		self.yaw = yaw
		# print("x", self.x)

class UTMStateCB():
	def __init__(self, sub_topic):
		self.state_command = None
		self.sub = rospy.Subscriber(sub_topic, Int8, self.utm_cb)

	def utm_cb(self, msg):
		self.state_command = msg.data

class AttitudePos():
    def __init__(self, attitude_sub,pos_sub,vel_cmd=None):
        self.sub = rospy.Subscriber(attitude_sub, Odometry, self.current_state)
        
        self.pos_sub = rospy.Subscriber(pos_sub, PoseStamped, self.pos_state)

        if vel_cmd != None:
            self.command_sub = rospy.Subscriber(vel_cmd, TwistStamped, self.command_vel)
        
        self.x_state = [0.0,0.0,0.0,0.0] #x, xdot, pitch, pitch_rate
        self.y_state = [0.0,0.0,0.0,0.0] #y, ydot, roll, roll_rate
    
        self.cmd_vel_x = [0.0, 0.0] #body x vel , pitch rate
        self.cmd_vel_y = [0.0, 0.0] #body y vel, roll rate

    def pos_state(self,msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        self.x_state[2] = pitch
        self.y_state[2] = roll

    def current_state(self, msg):
        """update current estimates"""
        px = msg.pose.pose.position.x 
        py = msg.pose.pose.position.y
        
        vel_x = msg.twist.twist.linear.x
        vel_y = msg.twist.twist.linear.y
        
        pitch_rate = msg.twist.twist.angular.x 
        roll_rate = msg.twist.twist.angular.y 
        
        #pitch_rate = pitch - self.x_state[2]/self.dt
        #roll_rate = roll - self.y_state[2]/self.dt
        self.x_state[0] = px
        self.x_state[1] = vel_x
        self.x_state[3] = pitch_rate

        self.y_state[0] = py
        self.y_state[1] = vel_y
        self.y_state[3] = roll_rate

    def command_vel(self, msg):
        self.cmd_vel_x[0] = msg.twist.linear.x 
        self.cmd_vel_x[1] = msg.twist.angular.x
        
        self.cmd_vel_y[0] = msg.twist.linear.y
        self.cmd_vel_y[1] = msg.twist.angular.y
       
       
if __name__ == '__main__':
	
	# wsl_ip = os.getenv('WSL_HOST_IP')
	# df = pd.read_csv(config.FILEPATH+config.FILENAME)
	
	uav_name = "uav0"
	quad_topic = "PX4_0/global_position/pose"
	quad = OdomLoc(quad_topic)

	#
	#tag_topic_filtered = uav_name+"/kf_tag/pose"
	tag_topic_filtered= "/displace/tag/pose"
	tagekf = OdomLoc(tag_topic_filtered)

	tag_raw_topic = uav_name+"/tag/pose"
	tag = OdomLoc(tag_raw_topic)

	att_topic = uav_name+"/mavros/odometry/in"
	pos_topic = uav_name+"/mavros/local_position/pose"
	cmd_topic = uav_name+"/mavros/setpoint_velocity/cmd_vel"
	attitude = AttitudePos(att_topic, pos_topic,  cmd_topic,)

	true_tag = "true_tag_dis"
	true_tag = OdomLoc(true_tag)
 	
	kf_name_list = []
	kf_est_list = []
 
	for i in range(0,8):
		kf_name = "/qe"+str(i)
		kf_name_list.append(kf_name)

		kf_tag_topic = kf_name+"/uav0/displace/tag/kfpose"
		kf_position =  OdomLoc(kf_tag_topic)

		p_error_var_topic = kf_name+"/uav0/displace/P_covar/pose"
		process_var = OdomLoc(p_error_var_topic)
  
		kf_est_list.append([kf_position, process_var])
		
	# for idx, uav in df.iterrows():
	# 	uav_name_list.append(uav['uav_name'])	
	# 	true_position_topic = uav['uav_name']+"/global_position/pose" 
	# 	#true_position_topic = "mavros/offset_local_position/pose"
	# 	true_position = OdomLoc(true_position_topic)
	# 	true_position_list.append(true_position)

	# relative_position_topic = "mavros/local_position/pose"
	# relative_position = OdomLoc(relative_position_topic)

	# utm_command_topic = "utm_control"
	# utm_command_state = UTMStateCB(utm_command_topic)
	
	# tag_raw_topic = "tag/pose"
	# tag = OdomLoc(tag_raw_topic)
	# register the node with the name logger
	rospy.init_node('logger', anonymous=True)


	#---------Logfile Setup-------------#
	# populate the data header, these are just strings, you can name them anything
	myData = ["time","quad x", "quad y", "quad z", "kftag x", "kftag y", 
           "kftag z", "tag x", "tag y", "tag z", "tag roll", "tag pitch",
           "true tag x", "true tag y",
           "pitch", "roll"]

	for kf in kf_name_list:
		myData.append(kf+"_position")
		myData.append(kf+"_error_covar")
		# myData.append(uav_name+"_x")
		# myData.append(uav_name+"_y")
		# myData.append(uav_name+"_z")

	# this creates a filename which contains the current date/time RaspberryPi does not have a real time clock, the files
	# will have the correct sequence (newest to oldest is preserved) but unless you set it explicitely the time will not
	# be the correct (it will not be the "real" time
	# the syntax for the command to set the time is:  bashrc: $ sudo time -s "Mon Aug 26 22:20:00 CDT 2019"
	# note that the path used here is an absolute path, if you want to put the log files somewhere else you will need
	# to include an updated absolute path here to the new directory where you want the files to appear
	fileNameBase = "/home/justin/catkin_ws/src/utm/utm/scripts/logfiles/" + "kf_q" + "_"+ datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + "kf_q" + fileNameSuffix
		num = num + 1

	# now we know we have a unique name, let's open the file, 'a' is append mode, in the unlikely event that we open
	# a file that already exists, this will simply add on to the end of it (rather than destroy or overwrite data)
	myFile = open(fileName, 'a')
	with myFile:
		writer = csv.writer(myFile)
		writer.writerow(myData)

	# get the CPU time at which we started the node, we will use this to subtract off so that our time vector
	# starts near 0
	zero_time = rospy.get_time()

	# this is some ros magic to control the loop timing, you can change this to log data faster/slower as needed
	# note that the IMU publisher publishes data at a specified rate (500Hz) and while this number could be
	# changes, in general, you should keep the loop rate for the logger below the loop rate for the IMU publisher
	rate = rospy.Rate(20) #100 Hz
	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():
			# get the current time and subtract off the zero_time offset
			now = (rospy.get_time()-zero_time)
			# create the data vector which we will write to the file, remember if you change
			# something here, but don't change the header string, your column headers won't
			# match the data
			myData = [now, quad.x, quad.y, quad.z, tagekf.x, tagekf.y, tagekf.z,
             tag.x, tag.y, tag.z, tag.roll, tag.pitch,
			 true_tag.x, true_tag.y, attitude.x_state[2], attitude.y_state[2]]
   
			for kf in kf_est_list:
				myData.append(kf[0].coords)
				myData.append(kf[1].coords)
    
			# stick everything in the file
			myFile = open(fileName, 'a')
			with myFile:
				writer = csv.writer(myFile)
				writer.writerow(myData)

			# this is ros magic, basically just a sleep function with the specified dt
			rate.sleep()

	# as stated before, try/except is used to nicely quit the program using ctrl+c
	except rospy.ROSInterruptException:
		pass


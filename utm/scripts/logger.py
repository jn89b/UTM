#!/usr/bin/env python

import rospy
import time
import csv
import os
import datetime
from std_msgs.msg import Int8

from utm import UAVGen
from geometry_msgs.msg import PoseStamped

# class OdomLoc():
#     def __init__(self,sub_topic):
#         self.x = None
#         self.y = None
#         self.z = None
#         self.sub = rospy.Subscriber(sub_topic, PoseStamped, self.odom_cb) 
    
#     def odom_cb(self,msg):
#         self.x = msg.pose.position.x
#         self.y = msg.pose.position.y
#         self.z = msg.pose.position.z

# class UTMStateCB():
# 	def __init__(self, sub_topic):
# 		self.state_command = None
# 		self.sub = rospy.Subscriber(sub_topic, Int8, self.utm_cb)

# 	def utm_cb(self, msg):
# 		self.state_command = msg.data
	

if __name__ == '__main__':
	uav_list = []
	for i in range(0,7):
		uav_name = "uav"+str(i)
		uav = UAVGen.UAVComms(uav_name)
		uav_list.append(uav)
	
	myData = ["time"]
	for uav in uav_list:
		myData.append(uav.name+"_coords")
		myData.append(uav.name+"_state")
		#myData.append(uav.name+"_state")

	# true_position_topic = "mavros/offset_local_position/pose"
	# true_position = OdomLoc(true_position_topic)

	# relative_position_topic = "mavros/local_position/pose"
	# relative_position = OdomLoc(relative_position_topic)

	# utm_command_topic = "utm_control"
	# utm_command_state = UTMStateCB(utm_command_topic)
	
	# tag_raw_topic = "tag/pose"
	# tag = OdomLoc(tag_raw_topic)
	# register the node with the name logger
	rospy.init_node('logger', anonymous=True)

	uav_id = rospy.get_param("~uav_id","uav")
	print(os.getcwd())
	#---------Logfile Setup-------------#
	# populate the data header, these are just strings, you can name them anything
	#myData = ["time", "uav_0_pos", "", "true_quad_z", "quad x", "quad y", "quad z", "utm command"]

	# this creates a filename which contains the current date/time RaspberryPi does not have a real time clock, the files
	# will have the correct sequence (newest to oldest is preserved) but unless you set it explicitely the time will not
	# be the correct (it will not be the "real" time
	# the syntax for the command to set the time is:  bashrc: $ sudo time -s "Mon Aug 26 22:20:00 CDT 2019"
	# note that the path used here is an absolute path, if you want to put the log files somewhere else you will need
	# to include an updated absolute path here to the new directory where you want the files to appear
	fileNameBase = "/home/justin/catkin_ws/src/utm/utm/scripts/logfiles/" + uav_id + "_"+ datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + uav_id + fileNameSuffix
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
			myData = [now]
			for uav in uav_list:
				myData.append(uav.mavros_coords)
				myData.append(uav.state_command)

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


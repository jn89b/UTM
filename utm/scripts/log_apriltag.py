#!/usr/bin/env python

import rospy
import time
import csv
import os
import datetime

from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

class OdomLoc():
    def __init__(self,sub_topic):
        self.x = None
        self.y = None
        self.z = None
        self.sub = rospy.Subscriber(sub_topic, PoseStamped, self.odom_cb) 
  
    def odom_cb(self,msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

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
        

uav_name = "uav0"
quad_topic = "PX4_0/global_position/pose"
quad = OdomLoc(quad_topic)

tag_topic_filtered = uav_name+"/kf_tag/pose"
tagekf = OdomLoc(tag_topic_filtered)

tag_raw_topic = uav_name+"/tag/pose"
tag = OdomLoc(tag_raw_topic)

true_tag = "/apriltag_global"
true_tag = OdomLoc(true_tag)

web_tag = "websling"
web = OdomLoc(web_tag)

att_topic = uav_name+"/mavros/odometry/in"
pos_topic = uav_name+"/mavros/local_position/pose"
cmd_topic = uav_name+"/mavros/setpoint_velocity/cmd_vel"
attitude = AttitudePos(att_topic, pos_topic,  cmd_topic,)

tag_dis_topic = uav_name+"/displace/tag/pose"
dist_tag = OdomLoc(tag_dis_topic)

kf_tag_dis_topic = uav_name+"/displace/tag/kfpose"
kf_tag_dis = OdomLoc(kf_tag_dis_topic)

# register the node with the name logger
rospy.init_node('logger', anonymous=True)
print(os.getcwd())
#---------Logfile Setup-------------#
# populate the data header, these are just strings, you can name them anything
myData = ["time","quad x", "quad y", "quad z", 
          "vel_x", "vel_y", "pitch", "roll", "pitch_rate", "roll_rate", 
          "kftag x", "kftag y", "kftag z", 
          "tag x", "tag y", "tag z", "true tag x", "true tag y",
          "cmd vel x", "cmd ang x", "cmd vel y", "cmd ang y",
          "web x", "web y", "dis x", "dis y", "dis kfx", "dis kfy"]

# this creates a filename which contains the current date/time RaspberryPi does not have a real time clock, the files
# will have the correct sequence (newest to oldest is preserved) but unless you set it explicitely the time will not
# be the correct (it will not be the "real" time
# the syntax for the command to set the time is:  bashrc: $ sudo time -s "Mon Aug 26 22:20:00 CDT 2019"
# note that the path used here is an absolute path, if you want to put the log files somewhere else you will need
# to include an updated absolute path here to the new directory where you want the files to appear
fileNameBase = "/home/justin/catkin_ws/src/utm/utm/scripts/logfiles/" + uav_name + "_"+ datetime.datetime.now().strftime("%b_%d_%H_%M")
fileNameSuffix = ".csv"
# num is used for incrementing the file path if we already have a file in the directory with the same name
num = 1
fileName = fileNameBase + fileNameSuffix
# check if the file already exists and increment num until the name is unique
while os.path.isfile(fileName):
	fileName = fileNameBase + "_" + str(num) + fileNameSuffix
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
rate = rospy.Rate(30) #100 Hz

if __name__ == '__main__':

	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():
			# get the current time and subtract off the zero_time offset
			now = (rospy.get_time()-zero_time)

			myData = [now, quad.x, quad.y, quad.z, 
             attitude.x_state[1], attitude.y_state[1], 
             attitude.x_state[2], attitude.y_state[2],
             attitude.x_state[3], attitude.y_state[3],
             tagekf.x, tagekf.y, tagekf.z, 
             tag.x, tag.y, tag.z,
             true_tag.x, true_tag.y,
             attitude.cmd_vel_x[0], attitude.cmd_vel_y[0],
             attitude.cmd_vel_x[1], attitude.cmd_vel_y[1],
             web.x, web.y, dist_tag.x, dist_tag.y, 
             kf_tag_dis.x, kf_tag_dis.y]
            
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

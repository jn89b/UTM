#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import setup_path
import airsim
import os
import re 
#from airsim.client import VehicleClient


if __name__=='__main__':
	wsl_ip = os.getenv('WSL_HOST_IP')
	client = airsim.VehicleClient(ip=str(wsl_ip))
	# client = airsim.VehicleClient(ip=str(172.23.112.1))
	client.confirmConnection()
	
	client.enableApiControl(True)
	#this is all in NED coordinates
	pose = airsim.Pose()
	pose.position.x_val = -30
	pose.position.y_val = 30
	pose.position.z_val = -25
	pose.orientation.x_val = 1
	pose.orientation.y_val = 0
	pose.orientation.z_val = 0
	pose.orientation.w_val = 1  
	
	#object_name, asset_name, pose, scale, physics_enabled=False, is_blueprint=False
	scale = airsim.Vector3r(1,1,1)
	test = client.simListAssets()
	#print("test", test)
	 #client.simSpawnObject('Hello', object ,pose,scale)
	n_range = [0,1,2,3,4,5,6,7,8,9]
	for n in n_range:
		string_name = (re.findall('\d+', 'PX4_'+str(n)))
		#get second number string values after PX4_
		waypoint_num = string_name[1]
		print('Waypoint_'+waypoint_num)
		for object in test:
			if object == 'Waypoint_'+waypoint_num:
				print("yes",object)	
				client.simSpawnObject('Hello', object ,pose,scale)
				#client.simDestroyObject('Hello',object ,pose,scale)
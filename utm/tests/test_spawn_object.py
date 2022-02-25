#!/usr/bin/env python
#
#import setup_path
import airsim
import os 
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
	for object in test:
		if object == 'Waypoint':
			print("yes",object)	
			client.simSpawnObject('Hello', object ,pose,scale)
	
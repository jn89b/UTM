#!/usr/bin/env python


#import setup_path
import airsim
import os
import re
import time 
#from airsim.client import VehicleClient


"""
lqr into wind: -9,-9
px4 into wind: 9, 9

lqr against wind: 9, 9
px4 against wind: -9,-9 
"""

if __name__=='__main__':
	wsl_ip = os.getenv('WSL_HOST_IP')
	client = airsim.VehicleClient(ip=str(wsl_ip))
	# client = airsim.VehicleClient(ip=str(172.23.112.1))
	client.confirmConnection()
	
	client.enableApiControl(True)
	
	wind_val = [0,0,0]
	print("adding wind ", wind_val[0],wind_val[1])
	#test 20 mph or 8.94m/s
	wind = airsim.Vector3r(wind_val[0], wind_val[1], 0)
	client.simSetWind(wind)
	
	# time.sleep(1.5)
	# wind_val = [0, -15, 0]
	# print("adding wind ", wind_val[0],wind_val[1])
	# #test 20 mph or 8.94m/s
	# wind = airsim.Vector3r(wind_val[0], wind_val[1], 0)
	# client.simSetWind(wind)

	iterations = 5
	wind_val = 8.5
	wind_1 = [0, -wind_val, 0]
	wind_2 = [0, wind_val, 0]
	wind_3 = [wind_val , 0, 0]
	wind_4 = [-wind_val , 0, 0]
	wind_5 = [wind_val , wind_val, 0]
	wind_6 = [-wind_val , -wind_val, 0]
	wind_7 = [-wind_val , wind_val, 0]
	wind_8 = [wind_val , -wind_val, 0]

	wind_list = [wind_1, wind_2, wind_3, wind_4,
              wind_5, wind_6, wind_7, wind_8]

	wind_val = [0, 0, 0]
	print("adding wind ", wind_val[0],wind_val[1])
	#test 20 mph or 8.94m/s
	wind = airsim.Vector3r(wind_val[0], wind_val[1], 0)
	client.simSetWind(wind)


	for wind in wind_list:
		print("adding wind ", wind[0],wind[1])
		#test 20 mph or 8.94m/s
		wind = airsim.Vector3r(wind[0], wind[1], 0)
		client.simSetWind(wind)
		time.sleep(3.0)
	
	time.sleep(1.5)
	wind_val = [0, 0, 0]
	print("wind sim done", wind_val[0],wind_val[1])
	#test 20 mph or 8.94m/s
	wind = airsim.Vector3r(wind_val[0], wind_val[1], 0)
	client.simSetWind(wind)
			
	
	# print("adding wind the other way", wind_val[0],wind_val[1])
	# #test 20 mph or 8.94m/s
	# wind = airsim.Vector3r(wind_val[0], wind_val[1], 0)
	# client.simSetWind(wind)
 
	#%% test spawn objects
	# #this is all in NED coordinates
	# pose = airsim.Pose()
	# pose.position.x_val = -30
	# pose.position.y_val = 30
	# pose.position.z_val = -25
	# pose.orientation.x_val = 1
	# pose.orientation.y_val = 0
	# pose.orientation.z_val = 0
	# pose.orientation.w_val = 1  
	
	# #object_name, asset_name, pose, scale, physics_enabled=False, is_blueprint=False
	# scale = airsim.Vector3r(1,1,1)
	# test = client.simListAssets()
	# #print("test", test)
	#  #client.simSpawnObject('Hello', object ,pose,scale)
	# n_range = [0,1,2,3,4,5,6,7,8,9]
	# for n in n_range:
	# 	string_name = (re.findall('\d+', 'PX4_'+str(n)))
	# 	#get second number string values after PX4_
	# 	waypoint_num = string_name[1]
	# 	#print('Waypoint_'+waypoint_num)
	# 	for object in test:
	# 		if object == 'AprilTag_Character2':
	# 			print("yes",object)	
	# 			client.simSpawnObject('Hello', object ,pose,scale)
	# 			#client.simDestroyObject('Hello',object ,pose,scale)
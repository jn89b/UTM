#!/usr/bin/env python
# -*- coding: utf-8 -*-

import airsim
import os 
#from airsim.client import VehicleClient

"""
testing out the airsim spawnobject pawn
"""
 
if __name__=='__main__':
    wsl_ip = os.getenv('WSL_HOST_IP')
    client = airsim.VehicleClient(ip=str(wsl_ip))
    client.confirmConnection()
    client.enableApiControl(True)
    print("spawning test pawn")
    #this is all in NED coordinates
    pose = airsim.Pose()
    pose.position.x_val = 10
    pose.position.y_val = -20
    pose.position.z_val = -15
    pose.orientation.x_val = 1
    pose.orientation.y_val = 0
    pose.orientation.z_val = 0
    pose.orientation.w_val = 1  
    
    #object_name, asset_name, pose, scale, physics_enabled=False, is_blueprint=False
    scale = airsim.Vector3r(1,1,1)
    test = client.simListSceneObjects()
    print(test)
    client.simSpawnObject("TestSpawn3","PX4_10.PX4_10",pose,scale,False)    #client.simSpawnObject('TestSpawn3','Cube',pose,scale)#,False,False)
    
    
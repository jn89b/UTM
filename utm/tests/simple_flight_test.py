#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import airsim 
import os

"""
Testing out how to fly 1 drone then multiple drones
this is a good reference
https://github.com/microsoft/AirSim/blob/master/PythonClient/multirotor/multi_agent_drone.py

Better thing to do in this situation is to have multiple ros nodes that listens to a topic name and 
goes to the command waypoints, input ENU -> Convert to NED  DONE

To do:
    - Have drone listen to waypoint list commands 
    - Goes to waypoints 
    - Can also precision land 
    - Can arm and disarm 


"""

def convert_enu_to_ned(enu_coords): 
    ned_x = enu_coords[1]
    ned_y = enu_coords[0]
    ned_z = -enu_coords[2]

    return [ned_x, ned_y, ned_z] 

def send_waypoint_commands():
    """send waypoint commands to UAVS"""

if __name__=='__main__':
    wsl_ip = os.getenv('WSL_HOST_IP')
# connect to the AirSim simulator
    client = airsim.MultirotorClient(ip=str(wsl_ip))
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    # Async methods returns Future. Call join() to wait for task to complete.
    client.takeoffAsync()# .join()
    #this moves in NED -> should switch to ENU for ros conventions?

    #this command gets all vehicle types will be useful for global planner
    vehicles = client.listVehicles()
    print("vehicles are", vehicles)
    # waypoints = [[30,50,10,5], [20,10,14,5]]

    # client.moveToPositionAsync(15, 0, -10, 5, vehicle_name=vehicles[0])

    # for wp in waypoints:
    #     ned_wp = convert_enu_to_ned(wp)
    #     async_call = client.moveToPositionAsync(ned_wp[0], ned_wp[1], ned_wp[2], wp[3],
    #      vehicle_name='PX4_0').join()

    #send waypoint commands

    

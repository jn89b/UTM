#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TO DO:
- Can arm/disarm
- set launch file for parameterization
- Can track fiducial tags - do this LAST
- listen for global commands waypoint commands - need to apply offsets.. 
- If we are doing swarm then local position too

- Should be able to query for waypoints multiple times

as a uas operator:
    - I want to be able to request waypoints so that I can send my uas to 
    places
    - I want to be able to notify that I have arrived to my destination, 
    so that the reservation table can be removed

AC:
    - Query Waypoints:
        - Send uav id, initial starting point, goal_point:
            - good situation: get waypoints
            - bad situation: no waypoints, return None
    - Arrived Destination:
        - After arrived to final waypoint, send uav name, to reservation table
            - good situation: query key from reservation and remove from reservation col
            - bad situation: wrong key so deny service
            - bad situation: don't remove from reservation table 
 
"""

import os
from re import L
import rospy
import airsim
import random
from geometry_msgs.msg import PoseStamped
from utm import Database
import numpy as np
import math as m
import re

def inflate_location(position, bounds):
    """inflate x,y,z locaiton position based on some bounds"""
    inflated_list = []
    #print("position is", position)
    """calculate bounds"""
    for i in bounds:
        for j in bounds:
            for k in bounds:
                new_position = [int(position[0]+i), int(position[1]+j), int(position[2]+k)]
                inflated_list.append(tuple(new_position))
                
    return inflated_list

class PID():
    """Compensator"""
    def __init__(self, kp, ki, kd, rate):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pre_error = 0.0    
        self.pre_ierror = 0.0
        self.rate = rate

    def compute_p_error(self, desired, actual):
        """get errors"""
        return desired - actual

    def __compute_i_error(self, error):
        """compute i gain with trapezoid rule"""
        return self.pre_ierror + ((error + self.pre_error)) * (1/self.rate)

    def __compute_d__error(self, error):
        """compute d gain"""
        return (error - self.pre_error)/(1/self.rate)

    def compute_gains(self, desired, actual):
        """return PID gains"""
        error = self.compute_p_error(desired, actual)
        i_error = self.__compute_i_error(error)
        d_error = self.__compute_d__error(error)

        gain = (self.kp*error)+ (self.ki*i_error) + (self.kd*d_error)

        self.pre_error = error
        self.pre_ierror = i_error

        return gain,error

class SimpleFlightDrone():
    """Control the SimpleFlight AirsimDrone"""
    def __init__(self, vehicle_name, wsl_ip, api_port):
        self.client = airsim.MultirotorClient(ip=str(wsl_ip), port=api_port)
        self.world_client =  airsim.VehicleClient(ip=str(wsl_ip), port=api_port)
        self.vehicle_name = vehicle_name
        
        self.global_pos_sub = rospy.Subscriber("/"+vehicle_name+"/global_position/pose", 
                                        PoseStamped,self.global_pos_cb)    
        
        self.offset_x = rospy.get_param("~offset_x", -10)
        self.offset_y = rospy.get_param("~offset_y", 20)

        self.get_start_position()
        self.get_goal_position()

        self.pid = PID(kp=0.75,ki=0.0,kd=0.0,rate=20)

        self.init_vel = rospy.get_param("~init_vel", 4)

        self.global_enu_pos = [None,None,None]
        self.global_enu_quat = [None,None,None, None]

        self.path_planning_service = Database.PathPlannerService()

        #this is bad need to take in the bubble as a parm
        self.col_bubble = 1
        self.col_radius = self.col_bubble/2
        self.bubble_bounds = list(np.arange(-self.col_radius, self.col_radius+1, 1))
        
        #this is also bad need to figure out how to parameterize what kind of bubble to use
        string_name = (re.findall('\d+', self.vehicle_name))
        #get second number string values after PX4_
        self.waypoint_num = string_name[1]
        
    def spawn_waypoints(self, enu_waypoints,index):
        """
        spawn waypoint assset with ned waypoint and ned orientation
        coordiats, need to refacor this code to keep track of names of 
        waypoints to remove once UAS has passed through it
        """
        #need to refactor
        ned_waypoint = self.convert_enu_to_ned(enu_waypoints)
        ned_orientation = [1,0,0,1]
        pose = airsim.Pose()
        pose.position = airsim.Vector3r(ned_waypoint[0], ned_waypoint[1], ned_waypoint[2])
        pose.orientation = airsim.Quaternionr(ned_orientation[0], ned_orientation[1],
                                              ned_orientation[2], ned_orientation[3])
        
        bubble_size = self.col_bubble * 0.8
        scale = airsim.Vector3r(bubble_size,bubble_size,bubble_size)
        self.client.simSpawnObject(self.vehicle_name+'_'+str(index), 
                                   'Waypoint_'+str(self.waypoint_num), pose, scale)

    def destroy_waypoint(self,index):
        """ 
        destroys waypoint of uav, need the object name, which will be the waypoint
        can't use TEST to make this work, will have to set it to name of vehicle
        """
        # scene_list = self.client.simListSceneObjects(self.vehicle_name+str(index))
        #if self.vehicle_name+str(index) in scene_list:
        self.client.simDestroyObject(self.vehicle_name+'_'+str(index))
        
    def get_start_position(self):
        """get starting position from params"""
        x = rospy.get_param("~init_x",  0)
        y = rospy.get_param("~init_y", 0)
        z = rospy.get_param("~init_z", 25)

        self.start_position = [x,y,z]

    def get_goal_position(self):
        """get goal position from params"""
        goal_x = rospy.get_param("~goal_x", 65)
        goal_y = rospy.get_param("~goal_y", 70)
        goal_z = rospy.get_param("~goal_z", 20)

        self.goal_position = [goal_x, goal_y, goal_z]

    def global_pos_cb(self, msg):
        """returns global ENU position of drone from subscribing to global 
        position topic"""
        enu_x = msg.pose.position.x
        enu_y = msg.pose.position.y
        enu_z = msg.pose.position.z

        enu_qx = msg.pose.orientation.x 
        enu_qy = msg.pose.orientation.y
        enu_qz = msg.pose.orientation.z
        enu_qw = msg.pose.orientation.w

        self.global_enu_pos = [enu_x, enu_y, enu_z]
        self.global_enu_quat = [enu_qx, enu_qy, enu_qz, enu_qw]
        
    def convert_enu_to_ned(self, enu_coords):
        """converts enu position to ned""" 
        ned_x = enu_coords[1]
        ned_y = enu_coords[0]
        ned_z = -enu_coords[2]
        #print("ned_coords",ned_x,ned_y,ned_z)
        return [ned_x, ned_y, ned_z]
    
    def convert_unreal_to_ned(self,unreal_coords):
        """convert unreal coordinates to ned coordinates"""
        return [unreal_coords[0], unreal_coords[1], -unreal_coords[2]]

    def armdisarm_drone(self, true_false):
        """arm or disarm drone based on true false condition"""
        self.client.armDisarm(true_false, self.vehicle_name)     

    def takeoff_drone(self):
        """commands drone to takeoff"""
        self.client.takeoffAsync(vehicle_name=self.vehicle_name)

    def init_drone(self):
        """initalize drone and send query to plan a path"""
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle_name)
        self.client.armDisarm(True, self.vehicle_name)
        self.takeoff_drone()
        self.send_enu_waypoint(self.start_position, self.init_vel)

    def compute_offsets(self,enu_wp):
        """send global commands have to subtract the offsets"""
        enu_global_x = enu_wp[0] - self.offset_x
        enu_global_y = enu_wp[1] - self.offset_y  
        enu_global_z = enu_wp[2] #- 0.8 #add these height offset because it can be weird
        return [enu_global_x, enu_global_y, enu_global_z]
 
    def spawn_waypoint_assets(self,enu_waypoints):
        """spawn all waypoints """
        for i, enu_wp in enumerate(enu_waypoints):
            if i % self.col_bubble == 0:
                self.spawn_waypoints(enu_wp,i)
            else:
                continue
            
    def send_enu_waypoints(self, enu_waypoints,velocity):
        """send a list of waypoints for drone to fly to"""
        self.spawn_waypoint_assets(enu_waypoints)
        
        for i,enu_wp in enumerate(enu_waypoints):
            
            self.send_enu_waypoint(enu_wp, velocity)
            
            if i % self.col_bubble == 0:
                #need to check if waypoints exist 
                self.destroy_waypoint(i)       
                
            #i have to do this because it simple flight won't hit the exact location
            if enu_wp == enu_waypoints[-1]:
                enu_wp = self.compute_offsets(enu_wp)
                ned_wp = self.convert_enu_to_ned(enu_wp)   
                print("ned waypoints are ", ned_wp)             
                self.moveToPosition(ned_wp,self.init_vel)
                if i % self.col_bubble == 0:
                    self.destroy_waypoint(i)    
        
                return True

    def send_enu_waypoint(self, enu_wp, velocity):
        """send enu waypoint command to drone by converting to ned to use api
        takes in the enu waypoint and the desired velocity"""
        
        ## spawn waypoints before computing offsets
        enu_wp = self.compute_offsets(enu_wp)
        ned_wp = self.convert_enu_to_ned(enu_wp)
        #join tells it to wait for task to complete
        #self.moveToPosition(ned_wp,velocity)
        self.client.moveToPositionAsync(ned_wp[0], ned_wp[1],
         ned_wp[2], velocity, vehicle_name=self.vehicle_name).join()
        
    def reinit_start(self):
        """reinitalize goal points"""
        self.start_position = self.goal_position

    def redefine_goal(self):
        """
        redefine goals based on a random number between bounds this is 
        done to simulate n queries for different goal points in the simulation
        distance_tol checks if the waypoint distance is long enough if not then
        keep searching
        """
        distance = 10
        distance_tol = 60
        lat_goal = [self.goal_position[0], self.goal_position[2]]
        while distance <= distance_tol: 
            x = random.randrange(20, 80)
            y = random.randrange(20, 80)
            z = random.randint(5, 45)
            distance = abs(m.dist([x,y], lat_goal))    
            if distance >= distance_tol:
                break
        
        self.goal_position = [x,y,z]

    def moveToPosition(self, desired_ned, v):
        """currentPos is in unreal engine coordinate from left hand convention"""
        currentPos = self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated.position        
        error = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 + (currentPos.z_val - desired_ned[2])**2)**0.5
        print("error is ", error)
        t = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 + (currentPos.z_val - desired_ned[2])**2)**0.5 / v
        tol = 0.5
        time_tol = 0.15
        while abs(error)>=tol:
            print("error is", error)
            if error <=tol or t <= time_tol: 
                print("im done")
                self.client.moveByVelocityAsync(0, 0, 0 , t, vehicle_name=self.vehicle_name)
                self.client.hoverAsync(vehicle_name=self.vehicle_name).join()
                rospy.sleep(t)
                return
            t = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 + (currentPos.z_val - desired_ned[2])**2)**0.5 / v
            x_gain,x_err = self.pid.compute_gains(desired_ned[0], currentPos.x_val)
            y_gain,y_err = self.pid.compute_gains(desired_ned[1], currentPos.y_val)
            z_gain,z_err = self.pid.compute_gains(desired_ned[2], currentPos.y_val)

            # delta_x = desired_ned[0] - currentPos.x_val
            # delta_y = desired_ned[1] - currentPos.y_val
            delta_z = desired_ned[2] - currentPos.z_val                
            vx = x_gain/t
            vy = y_gain/t
            vz = delta_z/t
                
            print("vx,vy,vz", [vx,vy,vz])
            self.client.moveByVelocityAsync(vx, vy, vz, t, vehicle_name=self.vehicle_name).join()
            rospy.sleep(t)
            currentPos = self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated.position        

            #rospy.sleep(t)
            error = ((currentPos.x_val - desired_ned[0])**2 + (currentPos.y_val - desired_ned[1])**2 + (currentPos.z_val - desired_ned[2])**2)**0.5

    def main(self):
        """main loop to request waypoints"""
        self.init_drone()
        rate_val = 20
        rate = rospy.Rate(rate_val)
        self.check_done = False
        
        self.path_planning_service.request_path(self.vehicle_name,
                                                self.start_position,
                                                self.goal_position)

        num_requests = 2
        i = 0
        while not rospy.is_shutdown():
            while i <= num_requests+1:
                waypoints_exist = self.path_planning_service.check_waypoints_exist(
                                                        self.vehicle_name,
                                                        self.start_position,
                                                        self.goal_position)
                
                
                if waypoints_exist:
                    waypoints = self.path_planning_service.get_uav_waypoints(
                                                        self.vehicle_name,
                                                        self.start_position,
                                                        self.goal_position)

                    if waypoints and self.check_done == False:
                        self.check_done = simple_drone.send_enu_waypoints(waypoints,self.init_vel)
                        goal_bounds = inflate_location(self.goal_position, self.bubble_bounds)
                        self.path_planning_service.remove_uav_from_reservation(self.vehicle_name, goal_bounds)
                        
                        i = i + 1
                        if i >= num_requests+1:
                            break

                        self.reinit_start()
                        self.redefine_goal()
                        self.path_planning_service.request_path(self.vehicle_name,
                                                        self.start_position,
                                                        self.goal_position)
                        
                        self.check_done = False
                # else:
                #     continue
            
            rate.sleep()

if __name__ == '__main__':
    
    rospy.init_node("simple_flight", anonymous=False)
    #should take in a ros param for veh_name
    veh_name = rospy.get_param("~veh_name", 'PX4_0')
    wsl_ip = os.getenv('WSL_HOST_IP')
    api_port = rospy.get_param("~api_port", 41451)
    
    simple_drone = SimpleFlightDrone(veh_name, wsl_ip, api_port)
    
    """waypoints are to be sent by the USS Path Planning Service """
    simple_drone.main()



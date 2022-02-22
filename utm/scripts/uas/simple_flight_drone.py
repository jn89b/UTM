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
import rospy
import airsim
import random
from geometry_msgs.msg import PoseStamped
from utm import Database

class SimpleFlightDrone():
    """Control the SimpleFlight AirsimDrone"""
    def __init__(self, vehicle_name, wsl_ip, api_port):
        self.client = airsim.MultirotorClient(ip=str(wsl_ip), port=api_port)
        self.vehicle_name = vehicle_name

        self.global_pos_sub = rospy.Subscriber("/"+vehicle_name+"/global_position/pose", 
                                        PoseStamped,self.global_pos_cb)    
        
        self.offset_x = rospy.get_param("~offset_x", -10)
        self.offset_y = rospy.get_param("~offset_y", 20)

        self.get_start_position()
        self.get_goal_position()

        self.init_vel = rospy.get_param("~init_vel", 20)

        self.global_enu_pos = [None,None,None]
        self.global_enu_quat = [None,None,None, None]

        self.path_planning_service = Database.PathPlannerService()

    def get_start_position(self):
        """get starting position from params"""
        x = rospy.get_param("~init_x", 0)
        y = rospy.get_param("~init_y", 5)
        z = rospy.get_param("~init_z", 15)

        self.start_position = [x,y,z]


    def get_goal_position(self):
        """get goal position from params"""
        goal_x = rospy.get_param("~goal_x", 65)
        goal_y = rospy.get_param("~goal_y", 70)
        goal_z = rospy.get_param("~goal_z", 20)

        self.goal_position = [goal_x, goal_y, goal_z]

    def global_pos_cb(self, msg):
        """returns global ENU position of drone"""
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
        return [ned_x, ned_y, ned_z]

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
        enu_global_z = enu_wp[2] - 0.8 #add these height offset because it can be weird
        return [enu_global_x, enu_global_y, enu_global_z]
 
    def send_enu_waypoints(self, enu_waypoints,velocity):
        """send a list of waypoints for drone to fly to"""
        for i,enu_wp in enumerate(enu_waypoints):
            self.send_enu_waypoint(enu_wp, velocity)
        return True

    def send_enu_waypoint(self, enu_wp, velocity):
        """send enu waypoint command to drone by converting to ned to use api
        takes in the enu waypoint and the desired velocity"""
        print("going to", enu_wp)
        enu_wp = self.compute_offsets(enu_wp)
        ned_wp = self.convert_enu_to_ned(enu_wp)
        #join tells it to wait for task to complete
        async_call = self.client.moveToPositionAsync(ned_wp[0], ned_wp[1],
         ned_wp[2], velocity, vehicle_name=self.vehicle_name).join()
        
    def reinit_start(self):
        """reinitalize goal points"""
        self.start_position = self.goal_position

    def redefine_goal(self):
        """redefine goals for uas"""
        x = random.randrange(25, 75)
        y = random.randrange(25, 75)
        z = random.randint(20, 50)
        self.goal_position = [x,y,z]

    def main(self):
        """main loop to request waypoints"""
        self.init_drone()
        rate_val = 20
        vel = 5
        rate = rospy.Rate(rate_val)
        self.check_done = False
        
        self.path_planning_service.request_path(self.vehicle_name,
                                                self.start_position,
                                                self.goal_position)

        num_requests = 5
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
                        self.check_done = simple_drone.send_enu_waypoints(waypoints,vel)
                        self.path_planning_service.remove_uav_from_reservation(self.vehicle_name)
                        i = i + 1
                        
                        if i >= num_requests+1:
                            break

                        self.reinit_start()
                        self.redefine_goal()
                        self.path_planning_service.request_path(self.vehicle_name,
                                                        self.start_position,
                                                        self.goal_position)
                        self.check_done = False
                else:
                    continue
            
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



#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TO DO:
- Can arm/disarm
- set launch file for parameterization
- Can track fiducial tags - do this LAST
- listen for global commands waypoint commands - need to apply offsets.. 
- If we are doing swarm then local position too
"""
import os
import rospy
import airsim
from geometry_msgs.msg import PoseStamped

class SimpleFlightDrone():
    """Control the SimpleFlight AirsimDrone"""
    def __init__(self, vehicle_name, wsl_ip, api_port):
        self.client = airsim.MultirotorClient(ip=str(wsl_ip), port=api_port)
        self.vehicle_name = vehicle_name

        self.global_pos_sub = rospy.Subscriber("/"+vehicle_name+"/global_position/pose", 
                                        PoseStamped,self.global_pos_cb)    
        
        self.offset_x = rospy.get_param("~offset_x", 30)
        self.offset_y = rospy.get_param("~offset_y", 20)

        self.init_x = rospy.get_param("~init_x", 0)
        self.init_y = rospy.get_param("~init_y", 0)
        self.init_z = rospy.get_param("~init_z", 10)

        self.init_vel = rospy.get_param("~init_vel", 10)

        self.global_enu_pos = [None,None,None]
        self.global_enu_quat = [None,None,None, None]

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

    def send_enu_waypoint(self, enu_wp, velocity):
        """send enu waypoint command to drone by converting to ned to use api
        takes in the enu waypoint and the desired velocity"""
        enu_wp = self.compute_offsets(enu_wp)
        ned_wp = self.convert_enu_to_ned(enu_wp)
        #join tells it to wait for task to complete
        async_call = self.client.moveToPositionAsync(ned_wp[0], ned_wp[1],
         ned_wp[2], velocity, vehicle_name=self.vehicle_name).join()

    def init_drone(self):
        self.client.confirmConnection()
        self.client.enableApiControl(True, self.vehicle_name)
        self.client.armDisarm(True, self.vehicle_name)
        self.takeoff_drone()
        self.send_enu_waypoint([self.init_x, self.init_y, self.init_z], self.init_vel)

    def compute_offsets(self,enu_wp):
        """send global commands have to subtract the offsets"""
        enu_global_x = enu_wp[0] - self.offset_x
        enu_global_y = enu_wp[1] - self.offset_y
        enu_global_z = enu_wp[2] - 0.8 #add these height offset because it can be weird
        return [enu_global_x, enu_global_y, enu_global_z]
 
    def send_enu_waypoints(self, enu_waypoints,velocity):
        """send a list of waypoints for drone to fly to"""
        for enu_wp in enu_waypoints:
            self.send_enu_waypoint(enu_wp, velocity)

        return True

if __name__ == '__main__':
    
    rospy.init_node("simple_flight", anonymous=False)
    #should take in a ros param for veh_name
    veh_name = rospy.get_param("~veh_name", 'PX4_0')
    wsl_ip = os.getenv('WSL_HOST_IP')
    api_port = rospy.get_param("~api_port", 41451)
    
    simple_drone = SimpleFlightDrone(veh_name, wsl_ip, api_port)
    simple_drone.init_drone()
    rate_val = 20
    vel = 5
    rate = rospy.Rate(rate_val)

    """waypoints are to be sent by the USS Path Planning Service """
    # enu_waypoint = [[-10,10,10], [30,15,10]]
    
    check_done = False
    while not rospy.is_shutdown():
        if check_done == False:
            print("going to waypoints")
            check_done = simple_drone.send_enu_waypoints(enu_waypoint,vel)
        else:
            pass

        rate.sleep()




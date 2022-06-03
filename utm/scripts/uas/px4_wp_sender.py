#!/usr/bin/env python

from this import d
import rospy 
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


"""
What does this do:
- query for waypoints
- send offboard user commands based on what it hears from database 
- send waypoints to px4 flightstack with unique number value, avoid duplicate
    -do this by sending list of lists and unique id number
- populate waypoint bubbles 
- if close to waypoint bubbles then remove the bubble 

"""
import os
from utm import Database
import airsim
import numpy as np
import math as m
import re
import pymongo
from utm.msg import WP, Coords

class DumbDrone():
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "landingServiceDatabase"
    landing_srv_col_name = "incoming_uas"
    
    def __init__(self, uav_name, position):
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]

        self.uav_name = uav_name
        self.pos = position
        
    def request_lz(self):
        """request landing zone from landing service database"""
        try:
            self.landing_service_col.insert_one({
                "_id": self.uav_name,
                "start_point": self.pos,
            })
        except pymongo.errors.DuplicateKeyError:
            # skip document because it already exists in new collection
            print("duplicate key", self.uav_name)

    def check_lz_found(self):
        """check if lz found for uav"""
        myquery = {"$and": [{"_id":self.uav_name, 
                    "start_point":self.pos}, 
                    {"goal_point": {'$exists': True}}]}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            return document['goal_point']

class PX4Helper():
    def __init__(self, vehicle_name, wsl_ip, api_port) -> None:
        self.path_planning_service = Database.PathPlannerService()
        self.world_client =  airsim.VehicleClient(ip=str(wsl_ip), port=api_port)
        self.vehicle_name = vehicle_name
        
        self.global_pos_sub = rospy.Subscriber("/"+vehicle_name+"/global_position/pose", 
                                        PoseStamped,self.global_pos_cb)    
        
        self.wp_pub = rospy.Publisher("uav0/wp_list", WP, queue_size=10)
        
        self.start_position = [30,50,70]
        self.global_enu_pos = [None,None,None]
        self.global_enu_quat = [None,None,None, None]
        self.goal_position = [None, None, None]
        
        self.lz_request = DumbDrone(vehicle_name, self.start_position)
        

    def publish_waypoints(self, enu_waypoints):
        #https://stackoverflow.com/questions/43130377/how-to-publish-subscribe-a-python-list-of-list-as-topic-in-ros
        wp_list = WP()
        wp_list.unique_wp_num = 0
        for i, wp in enumerate(enu_waypoints):
            coords = Coords()
            coords.data = [float(val) for val in wp]
            wp_list.data.append(coords)
            
        
        self.wp_pub.publish(wp_list)

    def spawn_waypoints(self, enu_waypoints,index):
        """
        spawn waypoint assets in unreal engine based on enu_waypoints and converts to
        ned waypoint and ned orientation
        coordinates, need to refacor this code to keep track of names of 
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
    
    def check_close(cur_position, des_position):
        """check if close to position"""
        
    def main(self):
        rate_val = 20
        rate = rospy.Rate(rate_val)
        
        self.lz_request.request_lz()
        goal = self.lz_request.check_lz_found()
        
        #begin the main loop right here
        while not rospy.is_shutdown():
            if goal != None:        
                waypoints_exist = self.path_planning_service.check_waypoints_exist(
                                                        self.vehicle_name,
                                                        self.start_position,
                                                        goal)
        
                if waypoints_exist:
                    waypoints = self.path_planning_service.get_uav_waypoints(
                                                        self.vehicle_name,
                                                        self.start_position,
                                                        goal)
                    # print(waypoints)
                    self.publish_waypoints(waypoints)
                else:
                    print("no waypoints")

            else:
                print("no waypoints")
        
        rate.sleep()
        
if __name__=='__main__':
    
    rospy.init_node("simple_flight", anonymous=False)
    #should take in a ros param for veh_name
    veh_name = rospy.get_param("~veh_name", 'PX4_1')
    wsl_ip = os.getenv('WSL_HOST_IP')
    api_port = rospy.get_param("~api_port", 41451)
    px4_help = PX4Helper(veh_name, wsl_ip, api_port)
    px4_help.main()

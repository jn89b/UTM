#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId
from utm import Database
from utm import HiearchialSearch
from utm import config

import airsim
import rospy
import os
import pickle
import mongodb_store_msgs.srv as dc_srv
import pymongo
import json
import numpy as np
import pandas as pd
        
def compute_actual_euclidean(position, goal):
    distance =  (((position[0] - goal[0]) ** 2) + 
                        ((position[1] - goal[1]) ** 2) +
                        ((position[2] - goal[2]) ** 2))**(1/2)
    
    return distance

def get_uav_names(dataframe):
    """return list of uav names from dataframe"""

    return df['uav_name'].to_list()


class USSPathPlanner(Database.PathPlannerService):
    def __init__(self):
        super().__init__()
        # self.world_client =  airsim.VehicleClient(ip=str(wsl_ip), port=41451)
        # self.world_client.confirmConnection()
        # self.world_client.enableApiControl(True)
    
    def init_reserve_table(self):
        """intialize reservation table queries for any reserved waypoints
        from collection and appends if there are any"""
        reserved = self.get_reserved_waypoints()
        #print("reserved is", reserved)
        if not reserved:
            print("reserved is empty")
            reservation_table = set()
        else:
            reservation_table = set()
            for waypoint in reserved:
                reservation_table.update(tuple(waypoint))
        return reservation_table
    
    def convert_enu_to_ned(self, enu_coords):
        """converts enu position to ned""" 
        ned_x = enu_coords[1]
        ned_y = enu_coords[0]
        ned_z = -enu_coords[2]
        #print("ned_coords",ned_x,ned_y,ned_z)
        return [ned_x, ned_y, ned_z]

    def set_start_goal(self, start, goal, bubble_bounds, reservation_table):
        """insert start and goal points into reservation table with its inflated 
        areas as well based on the collision bubble"""
        HiearchialSearch.insert_desired_to_set(start, reservation_table)
        HiearchialSearch.insert_inflated_waypoints(
            start, bubble_bounds, reservation_table)

        HiearchialSearch.insert_desired_to_set(goal, reservation_table)
        HiearchialSearch.insert_inflated_waypoints(
            goal, bubble_bounds, reservation_table)
    
    def deconflict_goals(self):
        """check if goal points are the same for different uavs 
        if so, then we need to deny the service"""
    
    def spawn_waypoint_assets(self,enu_waypoints, vehicle_name):
        """spawn all waypoints """
        for i, enu_wp in enumerate(enu_waypoints):
            if i % 5 == 0:
                self.spawn_waypoint(enu_wp,i, vehicle_name)
            else:
                continue
            
    def spawn_waypoint(self, enu_waypoints,index, vehicle_name):
        """
        spawn waypoint assset with ned waypoint and ned orientation
        coordiats, need to refacor this code to keep track of names of 
        waypoints to remove once UAS has passed through it
        """
        #need to refactor
        print("spawning")
        ned_waypoint = self.convert_enu_to_ned(enu_waypoints)
        ned_orientation = [1,0,0,1]
        pose = airsim.Pose()
        pose.position = airsim.Vector3r(ned_waypoint[0], ned_waypoint[1], ned_waypoint[2])
        pose.orientation = airsim.Quaternionr(ned_orientation[0], ned_orientation[1],
                                              ned_orientation[2], ned_orientation[3])
        scale = airsim.Vector3r(1,1,1)
        self.world_client.simSpawnObject(vehicle_name+str(index), 'Waypoint', pose, scale)


    def prioritize_uas(self,uav_list):
        """Takes in start list, and goal list and 
        prioritizes UAS based on highest distance to be traversed
        this should be move to the uss service
        
        uav has input of :
        uav[0] = name
        uav[1] = start point
        uav[2] = emnd point
        
        """
        
        dist_list = []
        for uav in uav_list:
            dist_val = compute_actual_euclidean(uav[1], uav[2])
            dist_list.append((dist_val,uav[1], uav[2], uav[0]))

        ##setting reverse to false sets to min first, true max first
        final_list = sorted(dist_list, key=lambda x: x[0], reverse=True)
        sorted_start = [start[1] for i, start in enumerate(final_list)]
        sorted_goal = [goal[2] for i, goal in enumerate(final_list)]
        sorted_uavs = [uav_name[3] for i, uav_name in enumerate(final_list)]

        return final_list, sorted_start, sorted_goal, sorted_uavs

    def check_waypoints_correct(self, waypoint_array, goal_point):
        """
        checks if final waypoint matches with the goal point
        has a built in try catch to see if waypoints is empty or not
        """
        waypoints = waypoint_array.tolist()
        
        try:        
            if waypoints[-1] == goal_point:
                return True
            else:
                print("not correct", waypoints[-1], goal_point)
                return False
            
        except IndexError:
            print("no waypoints", waypoints)
            return False
                
    def main(self):
        """main implementation"""
        """test if I have any clients"""
        rate_val = 20
        rate = rospy.Rate(rate_val)
        interval_time = 5.0
        print("starting")
        while not rospy.is_shutdown():
            #rospy.sleep(interval_time) #waiting for more queries 
            uav_list = self.find_path_planning_clients()
            
            if uav_list:
                final_list,sorted_start, sorted_goal, uav_name = self.prioritize_uas(uav_list)  
                
                ####-------BEGIN SEARCH, need to decouple this
                col_radius = col_bubble/2
                bubble_bounds = list(np.arange(-col_radius, col_radius+1, 1))
                
                reservation_table = self.init_reserve_table()            
                
                self.set_start_goal(sorted_start, sorted_goal,
                                    bubble_bounds, reservation_table)
                
                for start,goal,uav_id in zip(sorted_start, sorted_goal, uav_name):
                    hiearch_search = HiearchialSearch.begin_higher_search(start,goal,
                                    graph, annotated_map._Map__grid, obst_coords,col_radius, weighted_h,
                                    reservation_table)
                            
                    uav_waypoints= hiearch_search
                    uav_waypoints = [list(ele) for ele in hiearch_search]
                    
                    inflated_list = HiearchialSearch.insert_inflated_waypoints(
                        uav_waypoints, bubble_bounds , reservation_table)
                    
                    ## put into database
                    waypoints = np.array(uav_waypoints).astype(int)
                    
                    ## check if waypoints match to goal
                    if self.check_waypoints_correct(waypoints, goal) == False:
                        print("no waypoints can be found")
                        continue
                    
                    #inflated = np.array(inflated_list).astype(int)
                    self.insert_uav_to_reservation(uav_id, inflated_list)
                    self.insert_waypoints(uav_id, waypoints.tolist())               
                                
            rate.sleep()


if __name__=='__main__':
    rospy.init_node("self", anonymous=False)
    
    wsl_ip = os.getenv('WSL_HOST_IP')
    df = pd.read_csv(config.FILEPATH+config.FILENAME)

    ###### MAP and Grid Need to make this as a configuration    
    # PARAMS
    x_size = 100
    y_size = 100
    z_size = 75
    
    z_obs_height = 1
    num_clusters = 4
    
    load_map = False
    load_graph = False
    save_information = True
    
    map_pkl_name = 'map_test.pkl'
    graph_pkl_name = 'test.pkl'
    
    if load_map == True:
        with open(map_pkl_name, 'rb') as f:
            annotated_map  = pickle.load(f)
    else:
        ##### CONFIGURATION SPACE
        annotated_map= HiearchialSearch.build_map(3, x_size, y_size, z_size, z_obs_height, num_clusters, 10)
             
    ####-------- GRAPH 
    """I need to cache this to a database and query it to reduce start up costs
    I should save the information about the ostacles as well or maybe annoted map"""
    if load_graph == True:
        random_obstacles  = annotated_map._static_obstacles
        graph = HiearchialSearch.Graph(annotated_map, load_graph, graph_pkl_name)
    else:    
        random_obstacles = HiearchialSearch.generate_random_obstacles(1, x_size, z_obs_height)
        graph = HiearchialSearch.Graph(annotated_map, load_graph, graph_pkl_name)
        graph.build_graph()    
        graph.build_intra_edges()        
        HiearchialSearch.set_obstacles_to_grid(grid=annotated_map, obstacle_list=random_obstacles)
    
    obst_coords = annotated_map._Map__obstacles 
    col_bubble = 6
    weighted_h = 10
     
    uss_path_planner = USSPathPlanner()  
    uss_path_planner.main()  

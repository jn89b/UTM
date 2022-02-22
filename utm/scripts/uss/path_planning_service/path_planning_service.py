#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId
from utm import Database
from utm import HiearchialSearch
from utm import config

import rospy
import os
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
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

if __name__=='__main__':
    rospy.init_node("path_planning_service", anonymous=False)
    
    wsl_ip = os.getenv('WSL_HOST_IP')
    df = pd.read_csv(config.FILEPATH+config.FILENAME)

    ####### MAP and Grid Need to make this as a configuration    
    ## PARAMS
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
    col_bubble = 4
    weighted_h = 10
    

    #Need to do class implementation here?
    path_planning_service = Database.PathPlannerService()
    """test if I have any clients"""
    rate_val = 20
    vel = 5
    rate = rospy.Rate(rate_val)
    interval_time = 5.0

    while not rospy.is_shutdown():
        #rospy.sleep(interval_time) #waiting for more queries 
        uav_list = path_planning_service.find_path_planning_clients()
        #print("uav list is", uav_list)
        if uav_list:
            
            final_list,sorted_start, sorted_goal, uav_name = path_planning_service.prioritize_uas(uav_list)  
            
            ####-------BEGIN SEARCH, need to decouple this
            col_radius = col_bubble/2
            bubble_bounds = list(np.arange(-col_radius, col_radius+1, 1))
        
            reserved = path_planning_service.get_reserved_waypoints()
            if not reserved:
                print("reserved is empty")
                reservation_table = set()
            else:
                reservation_table = set()
                for waypoint in reserved:
                    reservation_table.update(tuple(waypoint))

            HiearchialSearch.insert_desired_to_set(sorted_start, reservation_table)
            HiearchialSearch.insert_inflated_waypoints(
                sorted_start, bubble_bounds, reservation_table)

            HiearchialSearch.insert_desired_to_set(sorted_goal, reservation_table)
            HiearchialSearch.insert_inflated_waypoints(
                sorted_goal, bubble_bounds, reservation_table)
            
            for start,goal,uav_id in zip(sorted_start, sorted_goal, uav_name):
                
                hiearch_search = HiearchialSearch.begin_higher_search(start,goal,
                                graph, annotated_map._Map__grid, obst_coords,col_bubble, weighted_h,
                                reservation_table)
                        
                uav_waypoints= hiearch_search
                uav_waypoints = [list(ele) for ele in hiearch_search]
            
                inflated_list = HiearchialSearch.insert_inflated_waypoints(
                    uav_waypoints, bubble_bounds , reservation_table)
                
                ## put into database
                waypoints = np.array(uav_waypoints).astype(int)
                #inflated = np.array(inflated_list).astype(int)
                path_planning_service.insert_uav_to_reservation(uav_id, inflated_list)
                path_planning_service.insert_waypoints(uav_id, waypoints.tolist())
            
            continue          
        else:
            #print("searching")
            continue

        rate.sleep()

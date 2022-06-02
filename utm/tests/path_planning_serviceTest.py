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

class UASTest():
    """
    test module of UAS requesting a waypoint
    """
    def __init__(self,uav_name, start, end):
        self.uav_name = uav_name
        self.start = start
        self.end = end

class PathPlannerService(Database.AbstractDatabaseInfo):
    """this is a test"""
    def __init__(self):
        pass

class PathPlannerService(Database.AbstractDatabaseInfo):
    """
    Path Planner Service takes in all information of UAS queries to request 
    a path to be sent from start point to goal point

    """
    database_name = "pathPlanningService"
    path_planning_col_name = "uas_path_planning"
    reservation_col_name = "reservation_table"

    ip = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    def __init__(self):
        super().__init__(self.ip,self.port_num,self.poolsize)
        self.mainDB = self.access_database(self.database_name)  

        #collection names
        self.path_planning_col = self.mainDB[self.path_planning_col_name]
        self.reservation_col = self.mainDB[self.reservation_col_name]

    def request_path(self,uav_name, start_point, end_point):
        """request path to db"""
        uav_info = {
                    "uav_name": uav_name,
                    "start_point": start_point,
                    "end_point": end_point
                    }

        self.path_planning_col.insert(uav_info)
                   
    def check_waypoints_exist(self,uav_name):
        """check if waypoints exist in database returns true if it does
        false if it doesnt"""
        myquery = {"waypoints": {'$exists': True}}
        cursor = self.path_planning_col.find(myquery)
        print("cursor is", cursor)
            
    def find_path_planning_clients(self):
        """find uas operators who do not have a waypoint and returns as list of lists"""
        uavs = []
        myquery = {"waypoints": {'$exists': False}}
        cursor = self.path_planning_col.find(myquery)
        for document in cursor:
            #print(document["uav_name"])
            uav = [document["uav_name"], document["start_point"],
            document["end_point"]]
            uavs.append(uav)

        return uavs
    
    def prioritize_uas(self,uav_list):
        """Takes in start list, and goal list and 
        prioritizes UAS based on highest distance to be traversed"""
        
        dist_list = []
        for uav in uav_list:
            dist_val = compute_actual_euclidean(uav[1], uav[2])
            print("distance val", dist_val)
            dist_list.append((dist_val,uav[1], uav[2], uav[0]))

        ##setting reverse to false sets to min first, true max first
        final_list = sorted(dist_list, key=lambda x: x[0], reverse=True)
        sorted_start = [start[1] for i, start in enumerate(final_list)]
        sorted_goal = [goal[2] for i, goal in enumerate(final_list)]
        sorted_uavs = [uav_name[3] for i, uav_name in enumerate(final_list)]

        return final_list, sorted_start, sorted_goal, sorted_uavs

    def insert_waypoints(self, uav_name, waypoint_list):
        """insert waypoints into path planning collection based on 
        uav name"""
        self.path_planning_col.update({"uav_name": uav_name},
        {"$set":{
            "waypoints": waypoint_list}})

        
def compute_actual_euclidean(position, goal):
    distance =  (((position[0] - goal[0]) ** 2) + 
                        ((position[1] - goal[1]) ** 2) +
                        ((position[2] - goal[2]) ** 2))**(1/2)
    
    return distance

def get_uav_names(dataframe):
    """return list of uav names from dataframe"""

    return df['uav_name'].to_list()

if __name__=='__main__':
    path_planning_service = PathPlannerService()
    wsl_ip = os.getenv('WSL_HOST_IP')
    df = pd.read_csv(config.FILEPATH+config.FILENAME)

    #print(path_planning_service.path_planning_col)
    #test inserting to waypoint
    # uav_name_test = "PX4_0"
    # uav_start = [4,2,5]
    # uav_end = [15,10,5]
    # uav_test = UASTest(uav_name_test, uav_start, uav_end)
    
    # for idx, uav in df.iterrows():
    #     uav_name = uav['uav_name']
    #     init_x = uav['init_x']
    #     init_y = uav['init_y']
    #     init_z = uav['init_z']
    #     goal_x = uav['goal_x']
    #     goal_y = uav['goal_y']
    #     goal_z = uav['goal_z']

    #     path_planning_service.request_path(uav_name, [init_x, init_y,
    #                                     init_z], [goal_x,goal_y, goal_z])

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
        #obst_coords = get_obstacle_coordinates(random_obstacles)
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
    
    """test if I have any clients"""
    uav_list = path_planning_service.find_path_planning_clients()
    
    if uav_list:
        final_list,sorted_start, sorted_goal, uav_name = path_planning_service.prioritize_uas(uav_list)
        
        ####-------BEGIN SEARCH
        hiearch_search = HiearchialSearch.begin_higher_search(sorted_start, sorted_goal,
                        graph, annotated_map._Map__grid, obst_coords,col_bubble, weighted_h)
        
        uav_waypoints= hiearch_search[1]

        for i, waypoints in enumerate(uav_waypoints):
            #print("uav name", uav_name[i])
            waypoints = [list(ele) for ele in waypoints]
            waypoints = np.array(waypoints).astype(int)

            path_planning_service.insert_waypoints(uav_name[i], waypoints.tolist())

    else:
        print("no uavs")

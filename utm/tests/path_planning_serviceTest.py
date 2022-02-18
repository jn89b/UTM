#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId
from utm import Database
from utm import HiearchialSearch

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
import pymongo
import json
import numpy as np


class UASTest():
    """
    test module of UAS requesting a waypoint
    """
    def __init__(self,uav_name, start, end):
        self.uav_name = uav_name
        self.start = start
        self.end = end

class PathPlannerService(Database.AbstractDatabaseInfo):
    """
    Path Planner Service takes in all information of UAS queries to request 
    a path to be sent from start point to goal point

    """
    database_name = "pathPlanningService"
    path_planning_col_name = "uas_path_planning"
    ip = "127.0.0.1"
    port_num = 27017
    poolsize = 100

    def __init__(self):
        super().__init__(self.ip,self.port_num,self.poolsize)
        self.mainDB = self.access_database(self.database_name)  

        #collection names
        self.path_planning_col = self.mainDB[self.path_planning_col_name]


    def request_path(self,uav_name, start_point, end_point):
        """request path to db"""
        uav_info = {#"_id": uav_name,
                    "uav_name": uav_name,
                    "start_point": start_point,
                    "end_point": end_point
                    }

        self.path_planning_col.insert(uav_info)
        print("updated collection with", uav_info)
                   
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

if __name__=='__main__':

    path_planning_service = PathPlannerService()
    
    #print(path_planning_service.path_planning_col)
    #test inserting to waypoint
    uav_name_test = "PX4_0"
    uav_start = [4,2,5]
    uav_end = [15,10,5]

    uav_test = UASTest(uav_name_test, uav_start, uav_end)
    path_planning_service.request_path(uav_name_test, uav_start, uav_end)

    """test if I have any clients"""
    uav_list = path_planning_service.find_path_planning_clients()

    #### MAP and Grid Need to make this as a configuration    
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

    # start_list = uav_list[1]
    # goal_list = uav_list[2]
    start_list = [uav_start]
    goal_list = [uav_end]
    
    obst_coords = annotated_map._Map__obstacles #i don't 
    col_bubble = 4
    weighted_h = 10
    
    ####-------BEGIN SEARCH
    hiearch_search = HiearchialSearch.begin_higher_search(start_list, goal_list,
                     graph, annotated_map._Map__grid, obst_coords,col_bubble, weighted_h)

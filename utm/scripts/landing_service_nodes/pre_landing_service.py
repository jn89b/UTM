#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

from utm import Database, HomeBase, PathFinding

from scipy import spatial
from queue import PriorityQueue

from mongodb_store_msgs.msg import StringPairList
from mongodb_store.message_store import MessageStoreProxy
from datetime import *

"""
To do:
get information of all open availible landing zones in the grida
get information of drones who are flying in and requesting service
get information of geofencing area in grid locations

plan flight for uav based on cost heuristic of minimal distance 
and no collisions

guide UAV to landing zone and set the open landing zone to closed 
set UAV state to 1 

listen for incoming UAVs and check if landing not avail

"""

class PreLandingService():
    """
    Pre Landing Service:
    Assigns Landing Zones with waypoints from 
    Should probably rename as Pre Flight Planner
    """
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "message_store"
    main_col_name = "data_service"
    landing_srv_col_name = "landing_service_db"
    landing_zone_col_name = "landing_zones"
    geofencing_col = None #need to figure out how to set up geofencing in the area
    
    def __init__(self):

        #access database
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        #collections 
        self.main_collection = self.mainDB[self.main_col_name]
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
        self.landing_zone_col = self.mainDB[self.landing_zone_col_name]

        #ros service proxies with mongodb
        self.data_srv_col_prox = MessageStoreProxy(collection=self.main_col_name)

        self.landing_srv_col_prox = MessageStoreProxy(collection=self.landing_srv_col_name)
        self.landing_zone_col_prox = MessageStoreProxy(collection=self.landing_zone_col_name)

    def check_open_zones(self):
        myquery = {"Vacant": True}
        cursor = self.landing_zone_col.find(myquery)
        if cursor.count() == 0:
            return False

    def find_open_zones(self):
        """requests query for open landing zones"""
        myquery = {"Vacant": True}
        open_zone_names = []
        open_zone_coordinates= []
        cursor = self.landing_zone_col.find(myquery)
        for document in cursor:
            #print(document['Zone Number'])
            open_zone_names.append(document['Zone Number'])
            open_zone_coordinates.append(tuple(document['location']))

        return open_zone_names, open_zone_coordinates

    def get_uavs(self):
        myquery = {"landing_service_status": 0}
        uav_names = []
        uav_battery = []
        cursor = self.landing_service_col.find(myquery)
        for document in cursor: 
            uav_names.append(document['uav_name'])
            uav_battery.append(document['battery'])
           
        return uav_names, uav_battery

    def get_uav_info(self, field_name):
        """return field name info where landing service status is at 0
        field_name must be type str"""

        myquery = {"landing_service_status": 0}
        uav_info_list = []
        cursor = self.landing_service_col.find(myquery)
        for document in cursor: 
            uav_info_list.append(document[field_name])

        return uav_info_list

    def find_closest_zone(self, uav_loc, landing_zones):
        """find closest zone location to uav location"""
        print("finding closest zone in", landing_zones)
        tree = spatial.KDTree(landing_zones)
        dist,zone_index = tree.query(uav_loc)

        return dist, zone_index

    def assign_uav_zone(self,uav_name, zone_name, uav_home_list):
        """assigns uav to zone and sets the landing zone as false, so no longer vacant"""
        self.landing_zone_col.update({"Zone Number": zone_name},
            { "$set": { 
                "Occupied by": uav_name,
                "Vacant": False }})

        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "Zone Assignment": zone_name,
                "Home Position": uav_home_list}})
        print("Assigned", uav_name + " to landing zone ", zone_name)

    def find_waypoints(self,grid_space, obstacles, uav_loc, goal_point):
        """sends the actual location in Unreal Engine coordiante axis
        so 5.0 is the actual 5.0 of the Unreal coordinate frame"""
        astar = PathFinding.Astar(grid_space, obstacles,  uav_loc, goal_point)
        uav_wp = astar.main()
        return uav_wp

    def insert_waypoints(self,uav_name, uav_waypoint_list):
        """insert filtered waypoint path for uav"""
        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "Waypoint": uav_waypoint_list}})

    def insert_raw_waypoints(self, uav_name, uav_raw_waypoint_list):
        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "Raw Waypoint": uav_raw_waypoint_list}})

    def get_offset_wp(self, uav_path, home_base_loc):
        """might not need this offset"""
        array = np.array(uav_path)
        result = (array[:,0] - home_base_loc[0], array[:,1] - home_base_loc[1], array[:,2])
        x = result[0]
        y = result[1]
        z = result[2]
        offset_wp = [list(coords) for coords in zip(x,y,z) ]
        
        return offset_wp

    def return_unassigned_list(self,some_list, index):
        """return all other zones or uavs not assigned to uav to make as a no fly zone"""
        copy = some_list
        copy.pop(index)
        print("copy", copy)
        return copy

    def add_obstacles(self,grid, obstacle_list):
        """"add obstacles to grid location"""
        for obstacle in obstacle_list:
            #print(obstacle)
            (grid[obstacle[2],obstacle[0], obstacle[1]]) = 1
            
        return obstacle_list

    def get_dynamic_obstacles(self, idx, uav_path_obs):
        """generate dynamic obstacles from uav waypoints"""
        #should be a function to make dynamic obstacles
        if idx == 0:
            new_obstacle = obstacle_list + \
                self.return_unassigned_list(zone_locations[:], zone_idx)
        else:
            uav_path_obs.append(path_list[idx-1])
            flat_list = [item for sublist in uav_path_obs for item in sublist]
            new_obstacle = obstacle_list + \
                self.return_unassigned_list(zone_locations[:], zone_idx) + \
                self.return_unassigned_list(uav_loc_list[:], idx) + flat_list
        grid_copy = grid.copy()
        new_obstacle = self.add_obstacles(grid_copy, new_obstacle)

        return grid_copy, new_obstacle

    def compute_vectors(self,point_1, point_2, point_3):
        vector_start = np.array(point_2)- np.array(point_1)
        vector_end = np.array(point_3) - np.array(point_2)
        
        return vector_start, vector_end
        
    def compute_cross_product(self,vector_1, vector_2):
        return np.cross(vector_1, vector_2)

    def reduce_waypoints(self,waypoint_list):
        print(waypoint_list)
        filtered_waypoints = []
        for i, waypoint in enumerate(waypoint_list):
            if i+2 - len(waypoint_list) == 0:
                filtered_waypoints.append(waypoint_list[i+1])
                """might want to append last waypoint value to new list"""
                return filtered_waypoints
            
            vec_start, vec_end = self.compute_vectors(waypoint, waypoint_list[i+1], waypoint_list[i+2])
            cross_product = self.compute_cross_product(vec_start, vec_end)
            if (cross_product[0] == 0 and cross_product[1] == 0
            and cross_product[2] == 0):
                print("collinear")
            else:
                print("not collinear")
                filtered_waypoints.append(waypoint)
                filtered_waypoints.append(waypoint_list[i+2])
                
        return filtered_waypoints

def generate_grid(grid_row, grid_col, grid_height):
    grid = []
    grid = np.zeros((grid_height, grid_row, grid_col))
    
    return grid

def plot_path(grid_z, grid_x, grid_y, waypoint_list, obstacles, goal):
    """plot pathway -> using this for flight trajectory"""
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.set_xlim([-1, grid_x])
    ax.set_ylim([-1, grid_y])
    ax.set_zlim([-1, 30])

    for obstacle in obstacles:
       ax.scatter(obstacle[0],obstacle[1], obstacle[2], color='red')
       
    #plot waypoints
    x = [x[0] for x in waypoint_list]
    y = [y[1] for y in waypoint_list]
    z = [z[2] for z in waypoint_list]
    ax.plot3D(x,y,z, 'bo', linestyle="-")
    ax.scatter(goal[0], goal[1], goal[2], color='purple', marker="+")

    plt.grid()
    plt.show()

if __name__ == '__main__':
    rospy.init_node("uav_sending_info")
    preLandingService = PreLandingService()

    """this is the homebase need to refactor this"""
    grid_z = 50 # this is probably the z axis
    grid_x = 50 # this is x
    grid_y = 50 # this is y
    grid = generate_grid(grid_z, grid_x,grid_y)
    
    static_obstacle_list = [(30,10)]
    obstacle_list = []
    for static_obstacle in static_obstacle_list:
        x = static_obstacle[0]
        y = static_obstacle[1]
        for z in range(25):
            obstacle_list.append((x,y,z))
    obstacle_list = preLandingService.add_obstacles(grid, obstacle_list)
    

    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        """this is very bloated need to refactor"""
        if preLandingService.check_open_zones() == False:
            print("No open zones waiting for uavs to leave")
        else:
            print("assigning")
            uav_path_obs = []
            path_list = []

            """probably better to refactor the information as a dictionary and 
            delete after its done doing its job"""
            uav_names = preLandingService.get_uav_info("uav_name")
            uav_loc_list = preLandingService.get_uav_info("uav_location")
            uav_home_list = preLandingService.get_uav_info("uav_home")

            """assigning locations"""
            for idx, uav_loc in enumerate(uav_loc_list):    
                zone_names, zone_locations = preLandingService.find_open_zones()
                dist, zone_idx = preLandingService.find_closest_zone(uav_loc, zone_locations)
                
                """generating obstacles"""
                grid_copy, new_obstacle = preLandingService.get_dynamic_obstacles(idx, uav_path_obs)
                    
                """apply astar algorithim here"""
                # astar = Astar(grid_copy, new_obstacle,  uav_loc, zone_locations[zone_idx])
                # uav_wp = astar.main()
                uav_wp = preLandingService.find_waypoints(grid_copy, new_obstacle, \
                    uav_loc, zone_locations[zone_idx])
                if uav_wp != None:

                    preLandingService.assign_uav_zone(uav_names[idx], zone_names[zone_idx], uav_home_list[idx])
                    path_list.append(uav_wp)
                    
                    """reduce the amount of waypoints we need to send"""
                    filter_wp = preLandingService.reduce_waypoints(uav_wp)
                    preLandingService.insert_waypoints(uav_names[idx], filter_wp)

                    """insert raw waypoints for path planning"""
                    preLandingService.insert_raw_waypoints(uav_names[idx], uav_wp)
                    """this plot is for debugging"""
                    #plot_path(grid_z, grid_x, grid_y, uav_wp, new_obstacle, zone_locations[zone_idx])
                else:
                    print("cant find waypoint for", uav_names[idx])
                    continue
        
        rate.sleep()

        
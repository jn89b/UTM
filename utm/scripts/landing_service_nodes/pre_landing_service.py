#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy

import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D 

from utm import Database, HomeBase

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

class UAV():
    """this is a fake uav has an id and position"""
    def __init__(self, id_name , position, index, goal):
        self.id = id_name
        self.starting_position = position 
        self.goalpoint = goal
        self.zone_index = index
        self.path = None
        self.offset_path = None

class Node():
    """
    parent = parent of current node
    posiition = position of node right now it will be x,y coordinates
    g = cost from start to current to node
    h = heuristic 
    f = is total cost
    """
    def __init__(self, parent, position):
        self.parent = parent
        self.position = position
        
        self.g = 0
        self.h = 0
        self.f = 0
        
    def __lt__(self, other):
        return self.f < other.f
    
    # Compare nodes
    def __eq__(self, other):
        return self.position == other.position

    # Print node
    def __repr__(self):
        return ('({0},{1})'.format(self.position, self.f))
    
class Astar():
    """Astar"""
    def __init__(self, grid, obs_list,start, goal):
        self.grid = grid
        self.start = [int(i) for i in start]
        print("start is", start)
        self.goal = goal
        print("goal is", goal)
        self.collision_bubble = 3.5
        self.height_boundary = 20
        self.ground_boundary = 5
        
        self.obstacle_list = obs_list

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
        #self.openset = []

    def is_collision(self,distance):
        """check if there is a collision if so return True"""
        if distance <= self.collision_bubble:
            return True
    
    def find_closest_obstacle(self, obstacles, current_position):
        """find closest obstacle from obstacle list, wrt current position"""
        tree = spatial.KDTree(obstacles)
        dist, obst_index = tree.query(current_position)   
        
        return dist, obst_index
    
    def init_node(self):
        start_node = Node(None,tuple(self.start))
        start_node.g = start_node.h = start_node.f = 0
        self.openset.put((start_node.f, start_node))
        #self.openset.append(start_node)
        self.end_node = Node(None, tuple(self.goal))
        self.end_node.g = self.end_node.h = self.end_node.f = 0

    def is_move_valid(self, node_position):
        """check if move made is valid if so then return True"""
        if (node_position[0] > (len(self.grid) - 1) or 
            node_position[0] < 0 or 
            node_position[1] > (len(self.grid)-1) or 
            node_position[1] < 0 or
            node_position[2] > self.height_boundary or
            node_position[2] < self.ground_boundary ):
            return False
    
    def is_target_close(self, position, goal):
        """refactor this, just have distance as input"""
        """check if we are close to target if so we remove the penalty heuristic for 
        flying high or low"""
        distance = self.compute_euclidean(position,goal)
        if distance <= 1.5:
            return True

    def compute_euclidean(self,position, goal):
        """compute euclidiean with position and goal as 3 vector component"""
        distance =  math.sqrt(((position[0] - goal.position[0]) ** 2) + 
                        ((position[1] - goal.position[1]) ** 2) +
                        ((position[2] - goal.position[2]) ** 2))
        
        return distance

    #This function return the path of the search
    def return_path(self, current_node, grid):
        path = []
        no_rows = len(grid)
        no_columns = len(grid)
        # here we create the initialized result maze with -1 in every position
        result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
        current = current_node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
        # Return reversed path as we need to show from start to end path
        path = path[::-1]
        start_value = 0
        # we update the path of start to end found by A-star serch with every step incremented by 1
        for i in range(len(path)):
            result[path[i][0]][path[i][1]] = start_value
            start_value += 1
            
        return path
    
    def main(self):
        ss = 1
        move  =  [[ss, 0, 0 ], # go forward
                  [ 0, -ss, 0], # go left
                  [ -ss, 0 , 0], # go backward
                  [ 0, ss, 0 ], #go right
                  [ss, ss, 0 ], #go forward right
                  [ss, -ss, 0], #go forward left
                  [-ss, ss, 0 ], #go back right
                  [-ss, -ss, 0], #go back left
                  [ 0, ss , ss], #go up z 
                  [ 0, ss, -ss]] # go down z
        
        self.init_node()
        count = 0 

        """main implementation"""
        while not self.openset.empty():
        #while len(self.openset) > 0:
            count = count + 1
            #print(count)
            if count >= 4000:
                print("iterations too much")
                return self.closedset
            
            if self.openset.empty():
                print("No more moves")
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            self.closedset[current_node.position] = current_node
               
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                #print("Goal reached", current_node.position)
                path = self.return_path(current_node, self.grid)
                print("success!", count)
                return path
  
            #move generation
            children = []
            for new_position in move:
                
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],  current_node.position[2] + new_position[2])

                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    #print("move is invalid")
                    continue
        
                # Make sure walkable terrain
                #print("checking node", node_position)
                if self.grid[node_position] != 0:
                    #print("not walkable")
                    continue
                
                #check collision bubble here
                dist, obst_index = self.find_closest_obstacle(self.obstacle_list, node_position)
                #print("checking", self.obstacle_list[obst_index])
                if self.is_collision(dist):
                    #print("collision")
                    continue
                
                #create new node
                new_node = Node(current_node, node_position)
                
                # put to possible paths
                children.append(new_node)
                    
            #check each children 
            for child in children:
                #check if children is already visited
                if child.position in self.closedset:
                    #print("Exists", child.position)
                    continue
                
                if abs(current_node.position[2] - child.position[2]) == 1:
                    penalty = 1.25
                    #print("penalty", penalty)
                else:
                    penalty = 1                                                 
                
                """Heuristic costs calculated here, this is using eucledian distance"""
                #print("child.position", child.position)
                if self.is_target_close(current_node.position, self.end_node):
                    #print("current_node", current_node.position)
                    child.g = current_node.g + 1
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    dynamic_weight = 0.5
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    #print(child.f)
                else:
                    #print(current_node.g)
                    child.g = current_node.g + 1
                    dynamic_weight = 1.5
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                
                #add to open set
                #print("putting in", child)
                self.openset.put((child.f, child))

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
        astar = Astar(grid_space, obstacles,  uav_loc, goal_point)
        uav_wp = astar.main()
        return uav_wp

    def insert_waypoints(self,uav_name, uav_waypoint_list):
        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "Waypoint": uav_waypoint_list}})

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
    
    """this is very bloated need to refactor"""
    if preLandingService.check_open_zones() == False:
        print("No open zones")
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
            preLandingService.assign_uav_zone(uav_names[idx], zone_names[zone_idx], uav_home_list[idx])
            
            """generating obstacles"""
            grid_copy, new_obstacle = preLandingService.get_dynamic_obstacles(idx, uav_path_obs)

            """apply astar algorithim here"""
            # astar = Astar(grid_copy, new_obstacle,  uav_loc, zone_locations[zone_idx])
              # uav_wp = astar.main()
            uav_wp = preLandingService.find_waypoints(grid_copy, new_obstacle, \
                uav_loc, zone_locations[zone_idx])
            path_list.append(uav_wp)
            
            """reduce the amount of waypoints we need to send"""
            filter_wp = preLandingService.reduce_waypoints(uav_wp)
            preLandingService.insert_waypoints(uav_names[idx], filter_wp) 
            
            """this plot is for debugging"""
            #plot_path(grid_z, grid_x, grid_y, uav_wp, new_obstacle, zone_locations[zone_idx])

        
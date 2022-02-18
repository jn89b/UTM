#!/usr/bin/env python
# -*- coding: utf-8 -*- 

from __future__ import print_function
import numpy as np
import math as m
import random
import os
import pandas as pd

from utm import Astar #, AstarGraph, AstarLowLevel
import itertools
from itertools import combinations, permutations, product
import time
import gc
import heapq
import pickle
from operator import add, sub
from timeit import default_timer as timer

class Map():
    """
    overall search space, hence why I call it a Map
    """
    def __init__(self, z_size, x_size, y_size, obstacle_list):        
        self.x_size = x_size
        self.y_size = y_size
        self.z_size = z_size
        self.offset_val = 1 # this is stupid but the amount of step sizes in the grid
        self.__grid = self.__generate_grid(z_size, x_size, y_size)
        self.__obstacles = obstacle_list
        
        
        self._static_obstacles = {}
        self.__dynamic_obstacles = {}
        self.entrance_dict = {}
        self.cluster_dict = {}
        #should rename to move cluster
        self.__move_cluster_dict = {'left_entrance': [-1,0],
                     'right_entrance':[1,0],
                     'lower_entrance': [0,-1],
                     'upper_entrance': [0,1]}
        
    def get_grid(self):
        """return grid"""
        return self.__grid
    
    def __generate_grid(self, z_size, x_size, y_size):
        """returns the grid size of the grid"""
        return np.zeros((z_size, x_size, y_size))
    
    def get_obstacle_list(self):
        return self.__obstacles

    def __insert_into_static_obstacle_db(self, obstacle):
        """appends object into static obstal"""
        self._static_obstacles[obstacle.x, obstacle.y] = obstacle

    def __check_valid_cluster_size(self, n_clusters):
        """check if the number of clusters to make equal squares 
        are valid"""
        if (n_clusters % 2) != 0:
            print("you need an even number")
            return False
        if ((self.x_size*self.y_size) % n_clusters) != 0:
            print("can't do this")
            return False
        else:
            return True

    def __get_cluster_bounds(self,index, step_size):
        """get min and max bounds of the cluster"""
        #print(index, index+step_size)
        return [index, index+step_size-self.offset_val] 

    def __compute_cluster_size(self, n_clusters):
        """compute the cluster size dimension of the grid"""
        return (self.x_size**2/n_clusters)
    
    def __get_cluster_edges(self, max_bound, cluster_space):
        """return cluster edges from the cluster space:
            left edge is [x_lower, length of cluster]
            right edge is [x_upper , length of cluster]
            size should be a 2D vector [max_bound , height of z] 
        """
        upper_edge = cluster_space[:, :, max_bound-self.offset_val]
        left_edge = cluster_space[:, 0,:]
        right_edge = cluster_space[:, max_bound-self.offset_val, :]
        lower_edge = cluster_space[:, :, 0]
        cluster_edges = {'left_edge': left_edge,
                         'right_edge': right_edge,
                         'upper_edge': upper_edge,
                         'lower_edge': lower_edge} 
        
        return cluster_edges
    
    def __get_opposite_direction(self, entrance_key):
        """return the index of the opposite direction"""
        self.opposite_direction = {'left_entrance': 'right_entrance',
                     'right_entrance':'left_entrance',
                     'lower_entrance': 'upper_entrance',
                     'upper_entrance': 'lower_entrance'}
        
        return self.opposite_direction[entrance_key]
    
    def get_adjacent_clusters(self, entrance_key, cluster_pos):
        """return adjacent cluster based on the current cluster position and the 
        move made"""
        move = self.__move_cluster_dict[entrance_key]
        adjacent_cluster_position= [cluster_pos[0] + move[0], cluster_pos[1] + move[1]]        
        opposite_entrance = self.__get_opposite_direction(entrance_key) 
        
        return adjacent_cluster_position, opposite_entrance
    
    def place_static_obstacle(self, obstacle):
        """place obstacles inside the grid"""
        self.__insert_into_static_obstacle_db(obstacle)
        for obstacle_z in obstacle.z_list:
            self.__grid[obstacle_z, obstacle.x, obstacle.y] = 1
    
    def __check_duplicate_entrances(self, cluster_coords, adjacent_cluster_position):
        """check if we have duplicate cluster coordinates between two points"""
        paired_clusters = cluster_coords, adjacent_cluster_position
        flipped_pair = adjacent_cluster_position, cluster_coords 
        if str(paired_clusters) in self.entrance_dict:
            print("exists")
            return True
        if str(flipped_pair) in self.entrance_dict:
            print("exists")
            return True
        return False
    
    def link_entrances(self):
        """find entrances that link together"""        
        for cluster_key, cluster in self.cluster_dict.items():
            mapped_entrances = cluster.mapped_entrances
            for entrance_key, entrance_list in mapped_entrances.copy().items():
                
                adjacent_cluster_position,  adjacent_side = self.get_adjacent_clusters(entrance_key, 
                                                       cluster.cluster_coords)
                
                if self.__check_duplicate_entrances(cluster.cluster_coords, adjacent_cluster_position) == True:
                    continue

                paired_clusters = cluster.cluster_coords, adjacent_cluster_position
                connections = [entrance_list, self.cluster_dict[str(adjacent_cluster_position)].mapped_entrances[adjacent_side],
                               cluster.cluster_coords, adjacent_cluster_position] 
                self.entrance_dict[str(paired_clusters)] = connections
                
    def prune_entrances(self):
        """remove outer edges of the spaces"""        
        max_len = m.sqrt(len(self.cluster_list))
        for cluster_key, cluster in self.cluster_dict.items():
            cluster_pos = cluster.cluster_coords
            
            for i, (entrance_key, move) in enumerate(self.__move_cluster_dict.items()):
                position = [cluster_pos[0] + move[0], cluster_pos[1] + move[1]]
                    
                if position[0] < 0 :
                    del cluster.mapped_entrances[entrance_key]
                
                if position[0] > max_len - 1:
                    del cluster.mapped_entrances[entrance_key]
                
                if position[1] < 0:
                    del cluster.mapped_entrances[entrance_key]
                
                if position[1] > max_len -1:
                    del cluster.mapped_entrances[entrance_key]
                    
    def reduce_entryways(self, n_numbers):
        """reduce the number of entryways, this is kind of stupid but for now
        set it to a hard number?"""
        for cluster, cluster_vals in self.cluster_dict.items():
            for entrance_side, entrances in cluster_vals.mapped_entrances.items():
                if n_numbers == 0:
                    continue
                for i, coordinates in enumerate(entrances[:]):
                    if (i % n_numbers) != 0 or (coordinates[2] % n_numbers) != 0:
                        #print("coordinates are", i, coordinates)
                        entrances.remove(coordinates)


    def abstract_space(self, n_clusters):
        """generate clusters based on some number you want does it into 
        square pieces for this situation"""
        #check if cluster size is feasible
        if self.__check_valid_cluster_size(n_clusters) == True:
            ## cluster size is n x n
            cluster_size = self.__compute_cluster_size(n_clusters)
            step_size = int(m.sqrt(cluster_size))
            count = 0
            cluster_i = 0
            self.cluster_list = []
            for i in range(0, self.x_size, step_size):
                cluster_j = 0
                for j in range(0, self.y_size, step_size):
                    
                    x_min_max = self.__get_cluster_bounds(i,step_size) #[x_min, x_max]
                    y_min_max = self.__get_cluster_bounds(j, step_size) #[y_min, y_max]
                    cluster_name = str([cluster_i,cluster_j]) 
                    
                    #[start:stop] had to add this stupid offset to make sure we append the right values
                    cluster_space = self.__grid[:,x_min_max[0]:x_min_max[1]+ self.offset_val,
                                                y_min_max[0]:y_min_max[1]+ self.offset_val]  
                    
                    cluster_edges = self.__get_cluster_edges(step_size,
                                                             cluster_space)
                    limits = (x_min_max, y_min_max) 
                    
                    self.cluster_dict[cluster_name] = Cluster(cluster_name,
                                                        limits,
                                                        cluster_edges,
                                                        cluster_space,
                                                        n_clusters = n_clusters,
                                                        cluster_coords = [cluster_i, cluster_j])
                    self.cluster_list.append([cluster_i,cluster_j])
                    cluster_j +=1
                    count +=1
                cluster_i +=1
            
        else:
            print("no bueno")
            
class Graph():
    def __init__(self, configuration_map, load_data, graph_pkl_name):
        """
        load data boolean gives me the condition to either load pre generated graph to reduce 
        start up costs
        """
        if load_data == True:
            """
            Need to refactor this line to allower the user to specify the pkl 
            file location
            """
            with open(graph_pkl_name, 'rb') as f:
                self.graph = pickle.load(f)
        else:
            self.graph = {}
        
        self.map = configuration_map
        self.level = 1
        
    def build_graph(self):
        """build graph"""
        #this builds the inter edges
        for neighbor_clusters, vals in self.map.entrance_dict.items():
            adj_locations1 = vals[0]
            adj_locations2 = vals[1]
            c1_loc = vals[2]
            c2_loc = vals[3]
            for loc1, loc2 in zip(adj_locations1, adj_locations2):
                
                node1 = AbstractNode(loc1, c1_loc)
                node2 = AbstractNode(loc2, c2_loc)
                
                self.__add_node(node1)
                self.__add_node(node2)
                self.__add_edge(node1, node2, 1, 1, "INTER")
                
    def build_intra_edges(self):
        """build intra edges within clusters 
        number of connectsion = entrace_con + (m-1)(n) 
        where m is the number of nodes in the current location and n is the 
        number of nodes in the adjacent side"""
        for idx, (cluster, cluster_vals) in enumerate(self.map.cluster_dict.items()):
            cluster_loc = cluster_vals.cluster_coords
            inner_connections = []
            inner_sets = []
            print("working with cluster", cluster_vals.cluster_coords)
            for edge_side, location in cluster_vals.mapped_entrances.items():
                inner_connections.append(location)
                
                #get all permuations for nodes on the same side
                same_side_combos = list(combinations(location, 2))
                for same_side in same_side_combos:
                    inner_sets.append(same_side)
            
            #add all the intra nodes between the inner sides so left and right, etc
            for r in itertools.chain(product(inner_connections[0], inner_connections[1])
                                     , product(inner_connections[1], inner_connections[0])):
                inner_sets.append(r)
                
            """comment one of the other the last one looks at all connections"""
            #intra_connections_list = self.__remove_coords_diff_depth(inner_sets)
            intra_connections_list = inner_sets
            
            #this is a test to reduce the grid
            if idx == 4:
                return intra_connections_list, inner_sets
            
            #add all the intra nodes between the adjacent nodes need to do Astar to find distance
            for intra_connections in intra_connections_list:
                intra_node1 = AbstractNode(intra_connections[0], cluster_loc)
                intra_node2 = AbstractNode(intra_connections[1], cluster_loc)
                
                config_space = self.map.cluster_dict[str(cluster_loc)].cluster_space
                config_bounds = self.map.cluster_dict[str(cluster_loc)].limits
                distance = self.compute_actual_euclidean(intra_node1.location, intra_node2.location)
                #distance = self.__search_for_distance(intra_node1, intra_node2, config_space, config_bounds)
                self.__add_edge(intra_node1, intra_node2, distance, 1, "INTRA")
                                
    def __search_for_distance(self, intra_node1, intra_node2, cluster_space, config_bounds):
        """search for distance between two intra nodes using Astar
        this is due for a refactoring
        """
        obstacle_coords = self.map.get_obstacle_list()
        
        #this is stupid I just need to apply an offset to make to the node positions
        x_bounds = [config_bounds[0][0], config_bounds[0][1]]
        x_offset = abs(x_bounds[0])
    
        y_bounds = [config_bounds[1][0], config_bounds[1][1]]
        y_offset = abs(y_bounds[0])
        
        #print("Starting and goal point is", intra_node1.location, intra_node2.location)
        start_position = [abs(intra_node1.location[0] - x_offset),
                          abs(intra_node1.location[1] - y_offset), intra_node1.location[2]]
        
        goal_position = [abs(intra_node2.location[0] - x_offset),
                          abs(intra_node2.location[1] - y_offset), intra_node2.location[2]]
        
        #print(x_bounds, y_bounds)
        #added a garbage collection to remove clutter since this function will be used 
        gc.collect()
        astar = Astar(cluster_space, obstacle_coords, start_position,
                      goal_position, 0.5 , 4.5)
    
        path_list = astar.main()
        #print("path list is", path_list[0])

        #return a stupid high value if I cant find a path otherwise return the actual one
        if path_list is None:
            return 1E100
        
        if isinstance(path_list[0],list):
            return self.__compute_total_distance(path_list[0])
        else:
            return 1E100
            
    def compute_actual_euclidean(self, position, goal):
        distance =  (((position[0] - goal[0]) ** 2) + 
                           ((position[1] - goal[1]) ** 2) +
                           ((position[2] - goal[2]) ** 2))**(1/2)
        
        return distance
        

    def __compute_total_distance(self, path):
        """compute total sum of distance travelled from path list"""
        #print("path array", path)
        path_array = np.diff(np.array(path), axis=0)
        segment_distance = np.sqrt((path_array ** 2).sum(axis=1))
        return np.sum(segment_distance)    

    def __add_edge(self, node1, node2, weight, level, node_type):
        """makes edge between two nodes"""
        node1.set_cost(weight)
        node2.set_cost(weight)
        
        node1.set_node_type(node_type)
        node2.set_node_type(node_type)
        
        #check if intra node connections exists
        if not str(node1.location) in self.graph:
            self.graph[str(node1.location)] = set([node1])
            
        if not str(node2.location) in self.graph:
            self.graph[str(node2.location)] = set([node2])
        
        self.graph[str(node1.location)].add(node2)
        self.graph[str(node2.location)].add(node1)
            
    def __add_node(self, node):
        """add node to search graph"""
        if str(node.location) in self.graph:
            self.graph[str(node.location)].add(node)
        else:
            self.graph[str(node.location)] = set([node])
    
    def __remove_coords_diff_depth(self, inner_sets):
        """we don't want to connect intraedges at different heights f or now...
        space complexity will increase -> will look back at this later"""
        for i, coordinate_set in enumerate(inner_sets[:]):
            coord1 = coordinate_set[0]
            coord2 = coordinate_set[1]
            if abs(coord1[2] - coord2[2]) >=1:
                #print("removing", coordinate_set)
                inner_sets.remove(coordinate_set)
        
        return inner_sets
   
    def determine_cluster(self, coordinates):
        """determine which cluster location the coordinate set is located at 
        coordinates have to be x,y,z, and makes anode based on this"""
        
        #weaker precondition stronger postcondition check
        if len(coordinates) != 3:
            print("the coordinate input must be a list of [x,y,z]")
            return None
        
        for cluster, cluster_vals in self.map.cluster_dict.items():
            #print(cluster_vals.cluster_coords)
            x_bounds, y_bounds = cluster_vals.limits
            #print(cluster_vals.limits)
            if coordinates[0] in range(x_bounds[0], x_bounds[1]+1) and coordinates[1] in range(y_bounds[0], y_bounds[1]+1):
                #print("yes", cluster_vals.cluster_coords)
                
                return cluster_vals.cluster_coords
     
    def __add_temp_edges(self, temp_node, node2, weight, level, node_type, key_name):
        """adds the temporary node and connects it with other nodes in the hashtable"""
        temp_node.set_cost(weight)
        node2.set_cost(weight)
        
        temp_node.set_node_type(node_type)
        node2.set_node_type(node_type)
        
        #check if intra node connections exists
        if not str(key_name) in self.graph:
            self.graph[str(key_name)] = set([temp_node])
            
        if not str(node2.location) in self.graph:
            self.graph[str(node2.location)] = set([node2])
        
        self.graph[str(key_name)].add(node2)
        self.graph[str(node2.location)].add(temp_node)
            
        
    def connect_to_border(self,node, key_name):
        """connect borders to the map, I should have this in the graph class but define the key value
        so set key to start and goal to make it temporary"""
        offset_limit = 10 #this is dumb should have this parameterized
        height_bounds = [node.location[2]-offset_limit, node.location[2]+offset_limit]
        mapped_entrances_start = self.map.cluster_dict[str(node.cluster_coord)].mapped_entrances
        
        for entrance, entrance_list in mapped_entrances_start.items():
            for entrance_loc in entrance_list:
                """do a check for ranges don't want to connect to areas at higher places"""
                if entrance_loc[2] > height_bounds[0] and entrance_loc[2] < height_bounds[1]:
                    config_space = self.map.cluster_dict[str(node.cluster_coord)].cluster_space
                    config_bounds = self.map.cluster_dict[str(node.cluster_coord)].limits
                    intra_node2 = AbstractNode(entrance_loc, node.cluster_coord)
                    distance = self.compute_actual_euclidean(node.location, intra_node2.location)
                    #distance = self.__search_for_distance(node, intra_node2, config_space, config_bounds)
                    #probably should refactor this 
                    self.__add_temp_edges(node, intra_node2, distance, 1, "INTRA", key_name)
                else:
                    continue

    def insert_temp_nodes(self, location, level, key_name):
        """insert temp nodes into the graph takes in the AbstractNode, level, 
        and hash key name to be inserted 
        - use methods to determine cluster
        - connect to border
        - set level
        """
        cluster_coords = self.determine_cluster(location)
        #print("cluster coords are", cluster_coords)
        temp_node = AbstractNode(location , cluster_coords)        
        self.connect_to_border(temp_node, key_name)
    
class AbstractNode():
    def __init__(self, location, cluster_coord):
        self.location = location
        self.cluster_coord = cluster_coord
        self.level = None
        self.node_type = None
    
    def set_cost(self,cost):
        """this is the edge cost"""
        self.cost = cost
        
    def set_node_type(self,node_type):
        if node_type == "INTER":
            self.node_type = "INTER"
        if node_type == "INTRA":
            self.node_type = "INTRA"
                            
    def __get_comparison(self):
        return (tuple(self.location), self.node_type)
            
    def __eq__(self, other):
        return (self.location, self.node_type) == (other.location, other.node_type)
        
    def __ne__(self, other):
        return (not self.__eq__(other))
    
    def __hash__(self):
        return hash(self.__get_comparison())
    
class Cluster():
    def __init__(self, cluster_name, limits, edges, cluster_space, n_clusters,
                 cluster_coords):
        """
        entrances that link to the next cluster/graph
        obstacles within this sector
        section of the grid
        """
        self.cluster_name = cluster_name 
        self.cluster_space = cluster_space
        self.limits = limits
        self.edges = edges
        self.mapped_entrances = self.return_mapped_entrances()
        self.n_clusters = n_clusters
        self.cluster_coords = cluster_coords
        
    def __get_2d_coords(self,condition,i, x_limits, y_limits):
        """return 2d coordinates of edge based on condition"""
        if condition == "left_edge":
            return [x_limits[0], y_limits[0]+i]
    
        if condition == "right_edge":
            return [x_limits[1], y_limits[0]+i]
        
        if condition == "lower_edge":
            return [x_limits[0]+i, y_limits[0]]
        
        if condition == "upper_edge":
            return [x_limits[0]+i, y_limits[1]]
        
    def __get_entrance_coords(self, condition,edge, x_limits, y_limits):
        """return list of entrance coords"""
        edge_row_len, edge_col_len = edge.shape
        entrance_coords = []
        for i in range(edge_col_len):
            #this is a check to see if I have an obstacle along the edge or frontier of the cluster
            z_height = edge_row_len
            if edge[0,i] == 1:
                continue 
            some_coord = self.__get_2d_coords(condition, i, x_limits, y_limits)
            
            for j in range(z_height+1): #this should be mapped to some three dimensional height
                three_d = [some_coord[0], some_coord[1], j]
                entrance_coords.append(three_d)
            
            #sort the entrance coords based on z
            entrance_coords.sort(key=lambda x:x[2])
        return entrance_coords
        
    def return_mapped_entrances(self):
        """return mapped locations of entrances the first column are 
        the possible entrances case switch is better for this situation
        I need to refactor this line of code
        """
        mapped_entrances = {}
        x_limits = self.limits[0]
        y_limits = self.limits[1]
        
        for edge_direction, edge in self.edges.items():
            edge_len, edge_height =  edge.shape ##row and column size
            
            if edge_direction == "left_edge":
                entrance_coords = self.__get_entrance_coords("left_edge", edge, x_limits, y_limits)
                mapped_entrances["left_entrance"] = entrance_coords
            
            if edge_direction == "right_edge":
                entrance_coords = self.__get_entrance_coords("right_edge", edge, x_limits, y_limits)
                mapped_entrances["right_entrance"] = entrance_coords
            
            if edge_direction == "lower_edge":
                entrance_coords = self.__get_entrance_coords("lower_edge", edge, x_limits, y_limits)
                mapped_entrances["lower_entrance"] = entrance_coords

            if edge_direction == "upper_edge":
                entrance_coords = self.__get_entrance_coords("upper_edge", edge, x_limits, y_limits)
                mapped_entrances["upper_entrance"] = entrance_coords

        return mapped_entrances


class Obstacle():
    """obstacle path to put into grid"""
    def __init__(self, x_loc, y_loc, z_init, z_final):
        self.x = x_loc
        self.y = y_loc  
        self.z_list = self.__generate_z_height(z_init, z_final)
        
    def __generate_z_height(self, z_init, z_final):
        """create list height of obstacle based on z init and z final
        use list comprehension later"""
        z_list = []
        ## added 1 to get it to end at the exact number
        for z in range(z_init, z_final+1):
            z_list.append(z)
            
        return z_list

def generate_random_obstacles(n_random, bound_lim, z_obs_height):
    """creating random x and y coordinates"""
    x_random = random.sample(population=range(bound_lim), k=n_random)
    y_random = random.sample(population=range(bound_lim), k=n_random)
    
    random_static_obstacles = []
    
    for x,y in zip(x_random, y_random):
        random_static_obstacles.append(Obstacle(x_loc=x, y_loc=y, z_init=0,\
                                                z_final=z_obs_height))
    
    return random_static_obstacles
    
def set_obstacles_to_grid(grid, obstacle_list):
    """define obstacles"""
    for obstacle in obstacle_list:
        grid.place_static_obstacle(obstacle)    

def get_obstacle_coordinates(obstacles):
    """return locations of obstacles"""
    obst_coords = []
    for obstacle in obstacles:
        for z in obstacle.z_list:
            obst_coords.append([obstacle.x, obstacle.y,z])
            
    return obst_coords

# https://stackoverflow.com/questions/4529815/saving-an-object-data-persistence
def save_object(obj, filename):
    with open(filename, 'wb') as outp:  # Overwrites any existing file.
        pickle.dump(obj, outp, pickle.HIGHEST_PROTOCOL)

def build_map(num_obstacles, x_size, y_size, z_size, z_obs_height, num_clusters, num_entryways):
    """builds map or configuration space"""
    random_obstacles = generate_random_obstacles(n_random=num_obstacles, bound_lim=x_size, z_obs_height=z_obs_height)
    annotated_map = Map(z_size, x_size, y_size, get_obstacle_coordinates(random_obstacles))
    annotated_map.abstract_space(num_clusters)
    annotated_map.prune_entrances()
    annotated_map.link_entrances()
    annotated_map.reduce_entryways(num_entryways)
    
    return annotated_map

def get_abstract_path(start_location, goal_location, reservation_table,graph):
    """plans abstract path based on start location, goal location and what is reservered
    returns the abstract path as a list
    """
    graph.insert_temp_nodes(start_location, 1, start_location)
    graph.insert_temp_nodes(goal_location, 1, goal_location)
    gc.collect()
    astar_graph = Astar.AstarGraph(graph.graph, reservation_table, start_location, goal_location, 0.01)
    abstract_pathways = astar_graph.main()
    return abstract_pathways

def get_refine_path(graph, abstract_path, reservation_table, obstacle_coords, 
                    col_bubble,weighted_h):
    """get refined path for locations -> should use a set for obst coords"""
    waypoint_coords = []
    iteration_cnt = 0
    search_cnt = 0
    
    if isinstance(abstract_path , int):
        return [],iteration_cnt,search_cnt
    
    if abstract_path is None:
        return [],iteration_cnt,search_cnt

    for i in range(len(abstract_path)):
        if i+1>= len(abstract_path):
            #print("final iteration", iteration_cnt)
            return waypoint_coords, iteration_cnt, search_cnt
        
        lowastar = Astar.AstarLowLevel(
            graph, reservation_table, obstacle_coords,
            abstract_path[i], abstract_path[i+1], col_bubble, weighted_h
            )
        
        waypoints= lowastar.main()
        if waypoints is not None:
            #get time complexity and space complexity
            iteration_cnt += waypoints[1]
            search_cnt += len(waypoints[2])
            #print("length of dictionary is", len(waypoints[2]))
        
        if not waypoints:
            return [],iteration_cnt,search_cnt
        
        if isinstance(waypoints[0], list): 
            waypoint_coords.extend(waypoints[0])
        else:
            #print("final iteration", iteration_cnt)
            return waypoint_coords, iteration_cnt, search_cnt
            
def add_to_reservation_table(path_list, path_reservation_table):
    """add paths to reservation list"""
    for path in path_list:
        path_reservation_table.add(tuple(path))  
        
def generate_random_uav_coordinates(radius, x_bounds, y_bounds,  z_bounds, n_coords, obst_set):
    """generates random coordinates for uavs based on x bound, y bound, z bound
    and how many uavs you want and their proximity to each other"""
    # Generate a set of all points within 200 of the origin, to be used as offsets later
    # There's probably a more efficient way to do this.
    n_coords
    deltas = set()
    for x in range(-radius, radius+1):
        for y in range(-radius, radius+1):
            for z in range(-radius, radius+1):
                if x*x + y*y+ z*z <= radius*radius:
                    if (x,y,z) in obst_set:
                        continue
                    else:
                        deltas.add((x,y,z))
                        
    randPoints = []
    excluded = set()
    i = 0
    while i<n_coords:
        x = random.randrange(x_bounds[0], x_bounds[1])
        y = random.randrange(y_bounds[0], y_bounds[1])
        z = random.randint(z_bounds[0], z_bounds[1])
        if (x,y,z) in excluded or (x,y,z) in obst_set: 
            continue
        randPoints.append([x,y,z])
        i += 1
        excluded.update((x+dx, y+dy, z+dy) for (dx,dy,dz) in deltas)
    
    return randPoints
            

def insert_desired_to_set(coord_list, reservation_table):
    # if len(start_list) == 1:
    #     reservation_table.update(tuple(start_list))
    # else:
    tuple_point = [tuple(coord) for coord in coord_list]
    reservation_table.update(tuple_point)

def begin_higher_search(start_list, goal_list, graph, grid, obst_coords,
                        col_bubble, weighted_h):
    """begin higher search for n uavs -> probably better to use a dataframe?"""
    reservation_table = set()
    overall_paths = []
    abstract_paths = []
    iter_cnt_list = []
    search_space_list = []
    time_list = []
    
    col_radius = col_bubble/2
    bubble_bounds = list(np.arange(-col_radius, col_radius+1, 1))
    
    insert_desired_to_set(start_list, reservation_table)
    insert_inflated_waypoints(start_list, bubble_bounds, reservation_table)
    
    insert_desired_to_set(goal_list, reservation_table)
    insert_inflated_waypoints(goal_list, bubble_bounds, reservation_table)
    
    cnt = 0
    for start, goal in zip(start_list, goal_list):
                    
        #determine regions of coordinates on the cluster
        cluster_start = graph.determine_cluster(start)
        cluster_goal = graph.determine_cluster(goal)
        
        #remove the values from the reservation table for now
        # if tuple(start) in reservation_table:
        reservation_table.remove(tuple(start))
        start_bubble = inflate_location(start, bubble_bounds)
        remove_inflate_waypoints(start_bubble, bubble_bounds, reservation_table)
        
        # if tuple(goal) in reservation_table:
        reservation_table.remove(tuple(goal))
        goal_bubble = inflate_location(goal, bubble_bounds)
        remove_inflate_waypoints(goal_bubble, bubble_bounds, reservation_table)
        
        #time code 
        start_time = timer()
        #check if in same region
        if cluster_start == cluster_goal:
            #print("same region")
            abstract_path = [start,goal]
            waypoint_coords, iter_cnt, search_cnt = get_refine_path(
                                    grid, abstract_path,reservation_table, obst_coords,
                                    col_bubble, weighted_h)
        else:
            graph.insert_temp_nodes(start, 1, start)
            graph.insert_temp_nodes(goal, 1, goal)
            abstract_path = get_abstract_path(start, goal, reservation_table, graph)
            waypoint_coords,iter_cnt, search_cnt = get_refine_path(
                                    grid, abstract_path,reservation_table, obst_coords, 
                                    col_bubble, weighted_h)
        
        end_time = timer()
        time_diff = end_time-start_time
        #print("time to find solution", time_diff)
        
        cnt+=1
        gc.collect()
        
        #need to add inflation so with each waypoint add +1 -> l collision bubble
        # for x, y, z then insert to set
        #add_to_reservation_table(waypoint_coords, reservation_table)
        insert_inflated_waypoints(waypoint_coords, bubble_bounds , reservation_table)
        abstract_paths.append(abstract_path)
        overall_paths.append(waypoint_coords)
        iter_cnt_list.append(iter_cnt)
        search_space_list.append(search_cnt)
        time_list.append(time_diff)
        
    return reservation_table, overall_paths, abstract_paths, iter_cnt_list, search_space_list, time_list
        
def find_total_denials(overall_paths):
    """return total denial of paths"""
    cnt = 0
    for path in overall_paths:
        if not path:
            cnt+=1
    
    return cnt

def compute_success_percent(overall_paths):
    """compute overall percent success"""
    denials = find_total_denials(overall_paths)
    
    return abs(len(overall_paths) - denials)/len(overall_paths)

def insert_inflated_waypoints(waypoint_list,bounds, reservation_table):
    """insert inflated waypoints"""
    for waypoint in waypoint_list:
        inflated_list = inflate_location(waypoint, bounds)
        reservation_table.update(inflated_list)
        
def inflate_location(position, bounds):
    """inflate x,y,z locaiton position based on some bounds"""
    inflated_list = []
    """calculate bounds"""
    for i in bounds:
        for j in bounds:
            for k in bounds:
                new_position = [int(position[0]+i), int(position[1]+j), int(position[2]+k)]
                inflated_list.append(tuple(new_position))
                
    return inflated_list

def remove_inflate_waypoints(waypoint_list,bounds, reservation_table):
    """remove inflated waypoints from reservation table"""
    for waypoint in waypoint_list:
        if waypoint in reservation_table:
            reservation_table.remove(waypoint)

def save_image(image_name, fig):
    """saves image"""
    image_format = 'svg' # e.g .png, .svg, etc.
    # image_name = 'myimage.svfg'
    
    fig.savefig('images/'+image_name+'.svg', format=image_format, dpi=1200)
#!/usr/bin/env python
# -*- coding: utf-8 -*- 
"""
Created on Wed Oct 20 16:14:14 2021

@author: jn89b
this is due for a refactor
Astar 
take in grid size
take in collision objects 
take in x amount of drones
"""
from queue import PriorityQueue
from scipy import spatial

import numpy as np
import collections
import heapq
import numpy as np 
#import matplotlib.pyplot as plt
import math as m
# import re,seaborn as sns
# from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
import time

import os
import glob
import csv
# import pandas as pd
# from IPython import display

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

    def __init__(self, grid, obs_list,start, goal, col_bubble, weight_factor):
        self.grid = grid
        self.grid_z, self.grid_x, self.grid_y = grid.shape
        self.start = start
        self.goal = goal
        #print("starting", start, goal)
        self.collision_bubble = col_bubble
        self.weight_factor = weight_factor
        # self.height_boundary = 20
        # self.ground_boundary = 5
        
        self.obstacle_list = obs_list

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}

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
        if (node_position[0] > (self.grid_x) or 
            node_position[0] < 0 or 
            node_position[1] > (self.grid_y) or 
            node_position[1] < 0 or
            node_position[2] > self.grid_z  or
            node_position[2] < 0 ):
            return False
    
    def is_target_close(self, position, goal):
        """check if we are close to target if so we remove the penalty heuristic for 
        flying high or low"""
        distance = self.compute_euclidean(position, goal)
        
        if distance <= 1.5:
            return True
        
    #This function return the path of the search
    def return_path(self,current_node, grid):
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
        waypoints = []
        for points in path:
            waypoints.append(points)
            
        return waypoints
    
    
    def compute_euclidean(self,position, goal):
        """compute euclidean distance"""
        distance =  m.sqrt(((position[0] - goal.position[0]) ** 2) + 
                           ((position[1] - goal.position[1]) ** 2) +
                           ((position[2] - goal.position[2]) ** 2))
        
        return distance
    
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
                  [ 0, ss, -ss],
                  [ 0, 0, -ss] #go up z 
                  ] # go down z
        
        self.init_node()
        
        count = 0 
        """main implementation"""
        while not self.openset.empty():
            count = count + 1
            
            if count >= 4000:
                print("iterations too much for regular Astar")
                return None, count, self.closedset 
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            self.closedset[current_node.position] = current_node
               
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                #print("Goal reached", current_node.position)
                path = self.return_path(current_node, self.grid)
                #print("success!", count)
                return path, count, self.closedset
  
            #move generation
            children = []
            for new_position in move:
                
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],  current_node.position[2] + new_position[2])
                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    #print("move is invalid", node_position)
                    continue
        
                # Make sure walkable terrain have to minus 1 because the way python indexes -> its stupid
                if self.grid[node_position[2]-1,node_position[0]-1, node_position[1]-1] != 0:
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
                    penalty = 1.05
                    #print("penalty", penalty)
                else:
                    penalty = 1                                                 
                
                """Heuristic costs calculated here, this is using eucledian distance"""
                #print("child.position", child.position)
                if self.is_target_close(current_node.position, self.end_node):
                    #print("current_node", current_node.position)
                    #print("target is close", current_node.position)
                    cost = self.compute_euclidean(current_node.position, child)
                    child.g = current_node.g + 1
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    dynamic_weight = 0.75
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    #print(child.f)
                else:
                    dynamic_weight = self.weight_factor 
                    cost = self.compute_euclidean(current_node.position, child)
                    #print(current_node.g)
                    child.g = current_node.g + 1
                    #dynamic_weight = 15
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    child.f = child.g + (child.h *penalty*dynamic_weight)

                #print("putting in", child)
                self.openset.put((child.f, child))
                

class AstarGraph():
    def __init__(self, graph, reservation_table, start_location, end_location, collision_bubble):
        self.graph = graph
        self.reservation_table = reservation_table
        
        self.reservation_list = list(reservation_table)
        
        self.start_location = start_location
        self.end_location = end_location
        self.collision_bubble = collision_bubble 
        self.openset = PriorityQueue() # priority queue
        self.closedset = {}
        self.iter_limit = 5000 #this is stupid should be a paramter
        

    def __init_nodes(self):
        """initialize start and end location nodes"""
        start_node = Node(None, self.start_location)
        start_node.g = start_node.h = start_node.f = 0
        self.openset.put((start_node.f, start_node))
        
        end_node = Node(None, self.end_location)
        end_node.g = end_node.h = end_node.f = 0
    
    def __check_at_goal(self, current_position):
        """check if we have arrived at the goal"""
        if current_position == self.end_location:
            return True


    def __find_closest_obstacle(self, obstacles, current_position):
        """find closest obstacle from obstacle list, wrt current position"""
        tree = spatial.KDTree(obstacles)
        dist, obst_index = tree.query(current_position)   
        
        return dist, obst_index

    def __return_path_to_goal(self, current_node):
        """return path to starts"""
        path_goal  = []
        current = current_node 
        while current is not None:
            path_goal.append(current.position)
            current = current.parent
        #reverse path
        path_goal = path_goal[::-1]
        #print("path to goal is", path_goal)
        
        return path_goal
    
    def __unpack_tuple_coordinates(self, tuple_coords):
        """return tuple coordinates as a list"""
        return [tuple_coords[0],tuple_coords[1],tuple_coords[2]]

    def __make_node(self, current_node, neighbor_node):
        """inserts a new potential node to my neighbor based on neighbor
        and references it to the current node as parent"""
        new_node = Node(current_node, neighbor_node.location)
        new_node.g = current_node.g + neighbor_node.cost
        new_node.h = self.__compute_euclidean(neighbor_node.location, self.end_location)
        new_node.f = new_node.g + new_node.h 
        
        return new_node
    
    def __compute_euclidean(self,position, goal):
        """compute euclidean distance as heuristic"""
        distance =  m.sqrt(((position[0] - goal[0]) ** 2) + 
                            ((position[1] - goal[1]) ** 2) +
                            ((position[2] - goal[2]) ** 2))
        
        return distance
    
    def is_collision(self,distance):
        """check if there is a collision if so return True"""
        if distance <= self.collision_bubble:
            return True
    
    def main(self):
        """main implementation"""
        self.__init_nodes()
        
        #setting a iter count to prevent it from running forever
        iter_count = 0
        while not self.openset.empty():
            iter_count = iter_count + 1
            #pop current node off
            cost,current_node = self.openset.get()
            
            if iter_count >= self.iter_limit:
                # iter count
                return iter_count
            
            if self.__check_at_goal(current_node.position):
                path_home = self.__return_path_to_goal(current_node)
                return path_home
            
            self.closedset[str(current_node.position)] = current_node
            
            current_node_position = self.__unpack_tuple_coordinates(current_node.position)
            neighbors = self.graph[str(current_node_position)]
            for neighbor in neighbors:
                
                if neighbor.location == current_node_position:
                    continue
                
                if str(neighbor.location) in self.closedset:
                    continue
                
                if neighbor.location[0:2] == current_node_position[0:2]:
                    continue
                
                if tuple(neighbor.location) in self.reservation_table:
                    continue
                
                #make new node
                new_node = self.__make_node(current_node, neighbor)
            
                #put to open set
                self.openset.put((new_node.f, new_node))
            
            iter_count +=1
            
class AstarLowLevel():
    """might need to refactor the original Astar to include some set things
    or inherit from original Astar and use polymorphism for weaker post condition
    stronger postcondition constraints
    
    Improvmeents:
        refactor heuristics by the following:
            do a gradient iteration
            change delta z from start to goal:
                if negative then we want to go up so set cost to go down higher
                if positive then we want to go down so set cost to up higher
    
    """
    def __init__(self, grid, reservation_table, obs_list,start, goal, col_bubble, weight_factor):
        self.grid = grid
        self.grid_z, self.grid_x, self.grid_y = grid.shape
        self.start = start
        self.goal = goal
        self.reservation_list = list(reservation_table)
        self.collision_bubble = col_bubble
        self.weight_factor = weight_factor
        self.reservation_table = reservation_table
        self.obstacle_list = obs_list

        self.openset = PriorityQueue() # priority queue
        self.closedset = {}

        if self.start[2] - self.goal[2] < 0: 
            self._determine_penalty = "going down"
        else:
            self._determine_penalty = "going up"
        # else:
        #     self._determine_penalty = "level"
        

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
        if (node_position[0] > (self.grid_x) or 
            node_position[0] < 0 or 
            node_position[1] > (self.grid_y) or 
            node_position[1] < 0 or
            node_position[2] > self.grid_z  or
            node_position[2] < 0 ):
            return False
    
    def is_target_close(self, position, goal):
        """check if we are close to target if so we remove the penalty heuristic for 
        flying high or low"""
        distance = self.compute_euclidean(position, goal)
        
        if distance <= 1.5:
            return True
        
    def __determine_penalty(self):
        """determines the penalty of the cost for changing height
        if want to end at higher position -> penalize going down
        if want to end at lower position -> penalize going up """
        
        if self.start[2] - self.goal[2] <= 0: 
            return "going down"
        else:
            return "going up"
        
    def __compute_penalty(self, current_z, new_z):
        """compute penalty"""
        diff_z = current_z - new_z 
        if self._determine_penalty == "going down" and diff_z > 0:
            return 1.15
        if self._determine_penalty == "going up" and diff_z < 0:
            return 1.15
        # if self._determine_penalty =="level":
        #     return 1
            
        return 1.0
        
    #This function return the path of the search
    def return_path(self,current_node, grid):
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
        waypoints = []
        for points in path:
            waypoints.append(points)
            
        return waypoints
    
    
    def compute_euclidean(self,position, goal):
        """
        compute euclidean distance
        """
        distance =  m.sqrt(((position[0] - goal.position[0]) ** 2) + 
                           ((position[1] - goal.position[1]) ** 2) +
                           ((position[2] - goal.position[2]) ** 2))
        
        return distance
    
    def get_moves(self, ss):
        """
        returns all 3d moves based on a step size
        """
        bounds = list(np.arange(-ss, ss+1, 1))
        move_list = []
        for i in bounds:
            for j in bounds:
                for k in bounds:
                    move_list.append([i,j,k])
        
        #remove the value that doesnt move
        move_list.remove([0,0,0])
        return move_list

    def main(self):
        ss = 1
        move = self.get_moves(ss)
        self.init_node()
        
        count = 0 
        """main implementation"""
        while not self.openset.empty():
            count = count + 1
            
            if count >= 5000:
                print("iterations too much for low level")
                return None, count, self.closedset 
            
            #pop node off from priority queue and add into closedset
            cost,current_node = self.openset.get()
            self.closedset[current_node.position] = current_node
               
            #check if we hit the goal 
            if current_node.position == self.end_node.position:
                #print("Goal reached", current_node.position)
                path = self.return_path(current_node, self.grid)
                #print("success!", count)
                return path, count, self.closedset
  
            #move generation
            children = []
            for new_position in move:
                
                node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1],  current_node.position[2] + new_position[2])
                
                # Make sure within range (check if within maze boundary)
                if self.is_move_valid(node_position) == False:
                    #print("move is invalid", node_position)
                    continue
        
                # Make sure walkable terrain have to minus 1 because the way python indexes -> its stupid
                if self.grid[node_position[2]-1,node_position[0]-1, node_position[1]-1] != 0:
                    #print("not walkable")
                    continue
                
                #check collision bubble here
                dist, obst_index = self.find_closest_obstacle(self.obstacle_list, node_position)
                #print("checking", self.obstacle_list[obst_index])
                if self.is_collision(dist):
                    #print("collision")
                    continue
                
                if tuple(node_position) in self.reservation_table:
                    #print("its in the low level reservation table")
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
                
                """refactor this, determine if I am going up or down then 
                adjust the heuristic"""
                penalty = self.__compute_penalty(
                    current_node.position[2],  child.position[2]) 
                
                """Heuristic costs calculated here, this is using eucledian distance"""
                #print("child.position", child.position)
                if self.is_target_close(current_node.position, self.end_node):
                    cost = self.compute_euclidean(current_node.position, child)
                    child.g = current_node.g + 1
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    dynamic_weight = 0.75
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                    #print(child.f)
                else:
                    dynamic_weight = self.weight_factor
                    cost = self.compute_euclidean(current_node.position, child)
                    child.g = current_node.g + 1
                    child.h = self.compute_euclidean(child.position, self.end_node)
                    child.f = child.g + (child.h *penalty*dynamic_weight)
                
                self.openset.put((child.f, child))
    
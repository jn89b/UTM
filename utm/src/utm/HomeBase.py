#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import numpy as np

class HomeBase():
    """
    class HomeBase has information about the location of the base
    as well as landing zones locations of the base
    """

    def __init__(self):
        self._max_height = 50
        self._width_size = 50
        self._length_size = 50
        self._grid = self.generate_grid(self._width_size,
                                self._length_size, self._max_height)

        self._offset_homebase_loc = [25,25]
        
        self._landing_zones = {"Zone_0": [20, 20],
                            "Zone_1": [30, 20]}

        self._static_obstacle_list = [(30,10)]
        some_list = []
        for static_obstacle in self._static_obstacle_list:
            x = static_obstacle[0]
            y = static_obstacle[1]
            for z in range(25):
                some_list.append((x,y,z))
        
        self._tall_obstacles = some_list
        self._tall_obstacles = self.add_obstacles(self._grid, self._tall_obstacles)

    def generate_grid(self,grid_row, grid_col, grid_height):
        """generate grid of perimeter around the area"""
        grid = []
        grid = np.zeros((grid_height, grid_row, grid_col))
        
        return grid

    def generate_tall_obstacles(self, static_obstacle_list, obstacle_height):
        """generate tall obstacles"""
        tall_obstacles = []
        for static_obstacle in static_obstacle_list:
            x = static_obstacle[0]
            y = static_obstacle[1]
            for z in range(obstacle_height):
                tall_obstacles.append((x,y,z))

        tall_obstacles = self.add_obstacles(self._grid, tall_obstacles)

        return tall_obstacles

    def add_obstacles(self,grid, obstacle_list):
        """"add obstacles to grid location"""
        for obstacle in obstacle_list:
            (grid[obstacle[2],obstacle[0], obstacle[1]]) = 1
            
        return obstacle_list



    

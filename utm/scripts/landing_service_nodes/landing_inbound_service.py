#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
import pymongo
from utm import Database
import numpy as np
from scipy import spatial

from datetime import *
import math
import rospy

"""
landing zones: 
zone 1 = 70,70
zone 2 = 70,85 
zone 3 = 85,85
zone 4 = 85,70
"""

HEIGHT_LOITER = 15
HOME = [80,80]
RAD = 3
bubble_bounds = list(np.arange(-RAD, RAD+1, 1))


def inflate_location(position, bounds):
    """inflate x,y,z locaiton position based on some bounds"""
    inflated_list = []
    #print("position is", position)
    """calculate bounds"""
    for i in bounds:
        for j in bounds:
            for k in bounds:
                new_position = [int(position[0]+i), int(position[1]+j), int(position[2]+k)]
                inflated_list.append(tuple(new_position))
    
    #put current postion in the list as
    position_int = [int(position[0]), int(position[1]), int(position[2])]            
    inflated_list.append(tuple(position_int))
    
    return inflated_list

def create_fence(zone_loc):
    """project a straight line based on HEIGHT LOITER"""
    # added 5 for some extra tolerance
    line_fence = [(zone_loc[0], zone_loc[1], int(i)) for i in range(0,HEIGHT_LOITER+5)]
    
    for f in line_fence:
        zone_fence = inflate_location(f, bubble_bounds)   

    return zone_fence

def compute_2d_euclidean(position, goal_position):
    """compute euclidiean with position and goal as 2d vector component"""
    distance =  math.sqrt(((position[0] - goal_position[0]) ** 2) + 
                    ((position[1] - goal_position[1]) ** 2))
    
    return distance

class LandingPlanningService():
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "landingServiceDatabase"
    main_col_name = "data_service"
    landing_srv_col_name = "incoming_uas"
    landing_zone_col_name = "landing_zones"
    geofencing_col = None #need to figure out how to set up geofencing in the area
    
    def __init__(self) -> None:
        #access database
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)

        self.main_collection = self.mainDB[self.main_col_name]
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]
        self.landing_zone_col = self.mainDB[self.landing_zone_col_name]

        #ros service proxies with mongodb
        self.zonePlanner = Database.ZonePlanner()
        self.path_planning_service = Database.PathPlannerService()

    def find_closest_zone(self, uav_loc, landing_zones):
        """find closest zone location to uav location"""
        if len(uav_loc) == 3:
            uav_loc = uav_loc[:2]
        tree = spatial.KDTree(landing_zones)
        dist,zone_index = tree.query(uav_loc)

        return dist, zone_index

    def insert_zone(self, zone_name, zone_loc, zone_fence):
        """initialize zones, set zones to true"""
        try:
            self.landing_zone_col.insert_one({
                "_id": zone_name,
                "zone_name": zone_name,
                "location": zone_loc,
                "Vacant": True, 
                "zone_fence": zone_fence
            })
            print("added zone", zone_name)
        except pymongo.errors.DuplicateKeyError:
            # skip document because it already exists in new collection
            print("duplicate key", zone_name)
            
    def find_open_zones(self):
        """requests query for open landing zones"""
        myquery = {"Vacant": True}
        open_zone_names = []
        open_zone_coordinates = []
        cursor = self.landing_zone_col.find(myquery)
        for document in cursor:
            open_zone_names.append(document['zone_name'])
            open_zone_coordinates.append(tuple(document['location']))
            
        return open_zone_names, open_zone_coordinates
    
    def set_zone_vacancy(self, zone_number, true_or_false):
        """set zone vacancy"""
        if type(true_or_false) != bool:
            print("I need a boolean")
        else:
            self.landing_zone_col.update({"_id": zone_number},
            {"$set":{
                "Vacant": true_or_false
            }})
            print("Landing ", zone_number + " is now " + str(true_or_false))

    def assign_zone(self, uav_name, zone_name, zone_loc):
        """assign zone"""
        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "landing_zone": zone_name,
                "goal_point": zone_loc}})        
        print("Assigned", uav_name + " to landing zone ", zone_name)
        
    def set_uav_inbound(self, uav_name):
        """assign zone"""
        self.landing_service_col.update({"_id": uav_name},
            { "$set": { 
                "uav_state": "inbound"}})        
        
    def request_waypoint(self, uav_name, start_position, goal_position):
        """request waypoint to uav to the path planning service"""
        self.path_planning_service.request_path(uav_name,
                                                start_position,
                                                goal_position)

    def check_waypoints_found(self, uav_name, start_position, goal_position):
        wp_exist = self.path_planning_service.check_waypoints_exist(
                                                        uav_name,
                                                        start_position,
                                                        goal_position)

        return wp_exist 
        
    def find_landing_clients(self):
        """find uas operators who do not have a landing zone and return list"""
        uavs = []
        myquery = {"landing_zone": {'$exists': False}}
        cursor = self.landing_service_col.find(myquery)
        for document in cursor:
            uav = [document["_id"], document["start_point"]]
            uavs.append(uav)

        return uavs
    
    def apply_zone_fence(self, zone_name, fence):
        """set up geofence to zone"""
        self.path_planning_service.insert_zone_to_reservation(zone_name, fence)

    def remove_zone_fence(self, zone_name):
        """remove geofence to zone from reservation table"""
        self.path_planning_service.remove_zone_from_reservation(zone_name)

    def prioritize_uavs(self, uavs):
        """prioritize uavs in vicinity"""
        uav_names = [uav[0] for uav in uavs]
        uav_locs = [uav[1] for uav in uavs]
        dist_list = [compute_2d_euclidean(uav[1], HOME) for uav in uavs]
        sorted_dist, sorted_uavs, sorted_locs = zip(*sorted(zip(
            dist_list, uav_names, uav_locs)))

        return sorted_uavs, sorted_locs 

if __name__ == '__main__':

    ZONE_1 = [70,70]
    zone_1_fence = create_fence(ZONE_1)

    ZONE_2 = [70,85]
    zone_2_fence = create_fence(ZONE_2)

    ZONE_3 = [85,85]
    zone_3_fence = create_fence(ZONE_3)

    ZONE_4 = [85,70]
    zone_4_fence = create_fence(ZONE_4)
    
    fences = [zone_1_fence, zone_2_fence, zone_3_fence, zone_4_fence]
    
    landing_service = LandingPlanningService()
    landing_service.insert_zone("zone_1", ZONE_1, zone_1_fence)
    landing_service.insert_zone("zone_2", ZONE_2, zone_2_fence)
    landing_service.insert_zone("zone_3", ZONE_3, zone_3_fence)
    landing_service.insert_zone("zone_4", ZONE_4, zone_4_fence)
    
    ## testing how to apply zone fences
    landing_service.apply_zone_fence("zone_1",zone_1_fence)
    landing_service.apply_zone_fence("zone_2",zone_2_fence)
    landing_service.apply_zone_fence("zone_3",zone_3_fence)
    landing_service.apply_zone_fence("zone_4",zone_4_fence)

    ## testing how to remove zone fences
    # landing_service.remove_zone_fence("zone_1")
    # landing_service.remove_zone_fence("zone_2")
    # landing_service.remove_zone_fence("zone_3")
    # landing_service.remove_zone_fence("zone_4")
    rospy.init_node("landing_service_node", anonymous=False)

    rate_val = 1
    rate = rospy.Rate(rate_val)
    
    """begin main loop"""
    while not rospy.is_shutdown():
        uavs = landing_service.find_landing_clients()
        if uavs:        
            """uav = [name, position]"""
            sorted_uavs, sorted_locs = landing_service.prioritize_uavs(uavs)
            sorted_uavs = list(sorted_uavs)
            sorted_locs = list(sorted_locs)        
            for uav_name, uav_loc in zip(sorted_uavs, sorted_locs):
                open_zone_names, open_zone_locs = landing_service.find_open_zones()
                
                if not open_zone_names:
                    print("no open zones")
                    break 
                
                _, zone_idx = landing_service.find_closest_zone(uav_loc, open_zone_locs)
                
                #added z paramter to zone location for loiter location
                zone_loc = open_zone_locs[zone_idx]
                zone_location = [zone_loc[0], zone_loc[1], HEIGHT_LOITER]
                
                landing_service.remove_zone_fence(open_zone_names[zone_idx])
                landing_service.request_waypoint(uav_name, uav_loc, zone_location)
                landing_service.assign_zone(uav_name, open_zone_names[zone_idx], zone_location)
                landing_service.set_uav_inbound(uav_name)
                landing_service.set_zone_vacancy(open_zone_names[zone_idx], False)
        else:
            print("no uavs")

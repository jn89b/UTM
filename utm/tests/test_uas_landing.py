#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from utm import Database
import pymongo

"""
simple test script to see if I can send UAS info to Landing Service Database
"""

class DumbDrone():
    ip_address = "127.0.0.1"
    port_num = 27017
    poolsize = 100
    database_name = "landingServiceDatabase"
    landing_srv_col_name = "incoming_uas"
    
    def __init__(self, uav_name, position):
        self.dbInfo = Database.AbstractDatabaseInfo(self.ip_address, self.port_num, self.poolsize)
        self.mainDB = self.dbInfo.access_database(self.database_name)
        self.landing_service_col = self.mainDB[self.landing_srv_col_name]

        self.uav_name = uav_name
        self.pos = position
        
    def request_lz(self):
        """request landing zone from landing service database"""
        try:
            self.landing_service_col.insert_one({
                "_id": self.uav_name,
                "start_point": self.pos,
            })
        except pymongo.errors.DuplicateKeyError:
            # skip document because it already exists in new collection
            print("duplicate key", self.uav_name)

if __name__=='__main__':
    #dumb drone 
    dumb_drone = DumbDrone("PX4_1", [30,50,70])
    dumb_drone.request_lz()
        
    dumb_drone = DumbDrone("PX4_3", [99,0,70])
    dumb_drone.request_lz()
    
    dumb_drone = DumbDrone("PX4_4", [0,99,70])
    dumb_drone.request_lz()    

    dumb_drone = DumbDrone("PX4_5", [0,30,90])
    dumb_drone.request_lz()    

    dumb_drone = DumbDrone("PX4_6", [50,30,70])
    dumb_drone.request_lz()
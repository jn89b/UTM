#!/usr/bin/env python
# -*- coding: utf-8 -*- 
import rospy
import multiprocessing
from utm import Database

class ZoneObserver():
    """
    Class ZoneObserver constantly looks at all uavs that have currently 
    landed, if the uav begins to leave then the zone observer 
    will listen to uav if it has left, if so then it will update landing zone as
    open

    to do:
        - generate list of uavs in landing zones
        - map their location and distances from the landing zone   
    """
    previous_service_number = 2
    update_service_number = 3

    def __init__(self):
        self.zonePlanner = Database.ZonePlanner()

    def find_uavs_who_have_homepath(self):
        """find uavs that have a service status of 0 but do not have
        a waypoint"""
        uavs = []
        myquery = {"$and": [{"landing_service_status":self.previous_service_number}, 
                    {"Home Waypoints": {'$exists': True}}]}
        cursor = self.zonePlanner.landing_service_col.find(myquery)
        for document in cursor:
            uavs.append(document["uav_name"])

        return uavs

    def observe_uavs(self,uav_class_list,uav):
        """observe uavs and see if they have left"""
        if not uav.coords():
            uav.get_uav_coords()

        zone_name = self.zonePlanner.find_uav_zone_name(uav.name)
        zone_coords = self.zonePlanner.find_zone_waypoints(zone_name)
        
        if self.zonePlanner.has_left_zone(zone_coords, uav.coords):
            print("uav has left", uav.name)
            self.zonePlanner.update_landing_zone(zone_name)
            uav_class_list.remove(uav)
            return

    def main(self):
        uavs = self.find_uavs_who_have_homepath()
        uav_class_list = self.zonePlanner.generate_publishers(uavs)
        
        rate_val = 10 
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            """need to check when the class is empty, if empty we listen for more drones"""
            #for i in range(len(uav_class_list)):
            if not uav_class_list: #if nothing then we continue to listen for uavs
                print("Waiting for uavs")
                rospy.sleep(2.0)
                uavs = self.find_uavs_who_have_homepath()
                print("uavs are", uavs)
                uav_class_list = self.zonePlanner.generate_publishers(uavs)
            else:
                threads = []
                for idx, uav in enumerate(uav_class_list[:]):
                    print(uav_class_list)
                    rospy.sleep(1.0) #wait for a couple of seconds
                    t = multiprocessing.Process(self.observe_uavs(uav_class_list, uav))
                    t.start()
                    threads.append(t)

                    if not uav_class_list:
                        break

                for t in threads:
                    t.join()
                    
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('zone_observer')
    zoneObserver = ZoneObserver()
    zoneObserver.main()

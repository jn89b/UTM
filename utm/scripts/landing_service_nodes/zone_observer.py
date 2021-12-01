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

    def observe_uavs(self,uav_class_list, uav):
        #uav.send_utm_state_command(self.previous_service_number)
        """need to open multiple threads and send waypoint commands for drone"""
        waypoint_list = self.zonePlanner.find_uav_home_waypoints(uav.name)
        zone_name = self.zonePlanner.find_uav_zone_name(uav.name)
    
        for wp in waypoint_list:
            if uav.wp_index > (len(waypoint_list)-2):
                self.zonePlanner.update_landing_zone(zone_name)
                uav_class_list.remove(uav)
                print(uav.name + "has left area")
                break

            waypoint = waypoint_list[uav.wp_index]
            
            """badly worded this is if we are at some assigned waypoint"""
            if self.zonePlanner.is_arrived_to_zone(waypoint, uav.coords) == False:
                continue
            else:
                uav.wp_index +=1

    def check_valid_uav(self,uav):
        if uav.coords != [None,None]:
            return True
     
    def main(self):
        uavs = self.find_uavs_who_have_homepath()
        uav_class_list = self.zonePlanner.generate_publishers(uavs)
        
        rate_val = 5 
        rate = rospy.Rate(rate_val)

        while not rospy.is_shutdown():
            """need to check when the class is empty, if empty we listen for more drones"""
            if not uav_class_list: #if nothing then we continue to listen for uavs
                uavs = self.find_uavs_who_have_homepath()
                uav_class_list = self.zonePlanner.generate_publishers(uavs)
            else:
                processes = []
                for idx, uav in enumerate(uav_class_list[:]):
                    #print("uavs", uav_class_list)
                    if self.check_valid_uav(uav):
                        #rospy.sleep(2.0) #wait for a couple of seconds
                        p = multiprocessing.Process(self.observe_uavs(uav_class_list, uav))
                        p.start()
                        processes.append(p)

                        if not uav_class_list:
                            break
                    else:
                        continue

                    for p in processes:
                        #wait until we finish before continuing 
                        p.join()

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('zone_observer')
    zoneObserver = ZoneObserver()
    zoneObserver.main()

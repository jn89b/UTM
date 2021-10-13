#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util
from mongodb_store.message_store import MessageStoreProxy
from mongodb_store_msgs.msg import StringPairList, StringPair
from geometry_msgs.msg import Pose, Point, Quaternion, String
import platform
if float(platform.python_version()[0:2]) >= 3.0:
    import io
else:
    import StringIO

"""
DB NODE
listens to incoming uav id 
if uav id does not exist in database:
    request query of uav information
    only update the uav position 
else uav id does not exist in database:
    log new information about uav

UAV information recorded in database: DONE
    -uav id
    -uav position information, global and local
    -uav battery
    -uav service request
    -uav state
    =bundle with id number


third party service sees what information it requests 
if it requests the service then will publish information/control 
of drone

Be able to remove UAV from database: DONE
    -request uav id number
    -get all documents of uav id number
    -remove all from database

http://docs.ros.org/en/kinetic/api/mongodb_store/html/message_store_proxy.html
"""

if __name__ == '__main__':

    rospy.init_node("recieve_db")

    rate = rospy.Rate(0.5)

    msg_store = MessageStoreProxy(collection='data_serivice')

    try:
        """get all information about uav"""
        #result = msg_store.query(StringPairList._type)
        single = msg_store.query_named("uav0", Pose._type)
        #print("single is:", single)

        #result = msg_store.query(StringPairList._type)
        #multiple = msg_store.query(StringPairList._type,{"name": {"$meta": "uav0"}})
        #print("multiple:", multiple)
        some_list = []
        i = 0
        """
        test query
        for item in msg_store.query(Pose._type):
            print("item is:", item)
            some_list.append(item)
            i = i + 1
            #print(item)
        """
        """
        get all values of uav set single = False
        for item in msg_store.query_named("uav0", Pose._type,single=False):
            print("item is:", item)
            some_list.append(item)
        """
        p = Pose(Point(0, 1, 2), Quaternion(3, 4,  5, 6))
        string_msg = String()
        
        for item,meta in msg_store.query_named("uav0", StringPairList._type, single=False):
            #print(item._id)
            string_msg.data = "hello world" 
            #print(item)
            pose_id = item.pairs[0].second
            point_id = item.pairs[1].second
            quat_id = item.pairs[2].second
            result_id = item.pairs[3].second
            #retrieve pose value
            pose_val = msg_store.query_id(item.pairs[0].second, Pose._type)
            
            #update pose of uav
            msg_store.update_id(pose_id, p)

            some_list.append(item)

            """delete ids for pose_id based on uav value"""
            """
            msg_store.delete(pose_id)
            msg_store.delete(point_id)
            msg_store.delete(quat_id)

            msg_store.delete(result_id)
            """
            
            """delete id bundle key of drone"""
            
            #print(meta.get('_id'))
            #id_num = meta.get('_id')
            #msg_store.delete(str(id_num))

            #print(item.query_id)
                    
        print(len(some_list))
        #print(some_list)

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


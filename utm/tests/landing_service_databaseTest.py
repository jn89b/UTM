#!/usr/bin/env python
# -*- coding: utf-8 -*- 
from __future__ import print_function
from bson.objectid import ObjectId

import rospy
import mongodb_store_msgs.srv as dc_srv
import mongodb_store.util as dc_util

from mongodb_store_msgs.msg import StringPairList, StringPair
from mongodb_store.message_store import MessageStoreProxy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Bool
from datetime import *
import platform

if float(platform.python_version()[0:2]) >= 3.0:
    import io
else:
    import StringIO


if __name__ == '__main__':
    """
    Listen for UAVs who are requesting service 
    if uav id is not in this database then append to database 
    -set the landing service state to 0 
    -get information about location
    Landing Service States:
        - 0 = Prelanding state
        - 1 = Landing state
        - 2 = Post Landing state
        - 3 = Left proximity     
    """
    rospy.init_node("landing_service_database")
    global_database = MessageStoreProxy(collection='landing_service_info')
    msg_store = MessageStoreProxy(collection='landing_service_info')
    
    try:
        pass
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


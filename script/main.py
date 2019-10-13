#!/usr/bin/env python
import rospy
import os 
import copy
import subprocess
import re
from std_msgs.msg import Float32, String
from nav_msgs.msg import OccupancyGrid
import numpy as np
import numpy.random as rn
import json
from datamanager import DataManager
from wifimap import WifiMap

def get_wifi_strength():
    result_ = subprocess.check_output(["iwconfig"], stderr = subprocess.STDOUT)
    result = ''.join(result_)
    tmp = re.findall(r'level=-\d+', result)[0]
    wifi_strength = float(tmp[-3:])
    return wifi_strength

if __name__=="__main__":
    rospy.init_node('wifi_publisher')
    pub = rospy.Publisher("wifi", String, queue_size = 1)
    wfm = WifiMap()
    dm = DataManager()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        pos_  = wfm.get_global_position()
        if pos_ is not None:
            pos = np.array(pos_)
            wifi_strength = get_wifi_strength()
            dm.push(pos, wifi_strength)

        msg = String(data = dm.dumps())
        pub.publish(msg)
        r.sleep()


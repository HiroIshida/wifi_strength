#!/usr/bin/env python
import rospy
import os 
import copy
import subprocess
import re
from std_msgs.msg import Float32

def get_wifi_strength():
    result_ = subprocess.check_output(["iwconfig"], stderr = subprocess.STDOUT)
    result = ''.join(result_)
    tmp = re.findall(r'level=-\d+', result)[0]
    wifi_strength = float(tmp[-3:])
    return wifi_strength

def main():
    rospy.init_node('wifi_strength_publisher')
    pub = rospy.Publisher("wifi_strength", Float32, queue_size = 1)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        wifi_strength = get_wifi_strength()
        msg = Float32(data = wifi_strength)
        pub.publish(msg)
        r.sleep()

main()

#!/usr/bin/env python
import rospy
import os 
import subprocess
import re
from std_msgs.msg import Float32

rospy.init_node('wifi_strength_publisher')

def get_wifi_strength():
    result_ = subprocess.check_output(["iwconfig"], stderr = subprocess.STDOUT)
    result = ''.join(result_)
    tmp = re.findall(r'level=-\d\d', result)[0]
    wifi_strength = float(tmp[-3:])
    return wifi_strength


pub = rospy.Publisher("wifi_strength", Float32, queue_size = 1)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    msg = Float32(data = get_wifi_strength())
    pub.publish(msg)




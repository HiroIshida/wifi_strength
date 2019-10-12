#!/usr/bin/env python
import rospy
import os 
import copy
import subprocess
import re
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
import tf

global msg_map
global msg_map_zeros
msg_map = None
msg_map_zeros = None

def callback_map(msg):
    global msg_map, msg_map_zeros
    msg_map = msg

    msg_map_zeros = copy.deepcopy(msg)

def get_global_position(trans):
    global msg_map
    if msg_map is None:
        return None

    pos_map = msg_map.info.origin.position
    x = - pos_map.x + trans[0]
    y = - pos_map.y + trans[1]
    return x, y

def create_debug_map_message(trans):
    ret = get_global_position(trans)
    if ret is None:
        return None
    x = ret[0]
    y = ret[1]

    global msg_map_zeros
    if msg_map_zeros is None:
        return None

    info = msg_map_zeros.info
    resol = info.resolution
    data = [0 for i in range(len(msg_map_zeros.data))]
    idx_x = int(x // resol)
    idx_y = int(y // resol)
    to_idx = lambda i, j: j * info.height + i
    for i in [idx_x - 50 + i for i in range(101)]:
        for j in [idx_y - 50 + j for j in range(101)]:
            idx = to_idx(i, j)
            data[idx] = 1.0
    msg_map_zeros.data = data
    return msg_map_zeros

def get_wifi_strength():
    result_ = subprocess.check_output(["iwconfig"], stderr = subprocess.STDOUT)
    result = ''.join(result_)
    tmp = re.findall(r'level=-\d+', result)[0]
    wifi_strength = float(tmp[-3:])
    return wifi_strength

def main():
    rospy.init_node('wifi_strength_publisher')
    #pub = rospy.Publisher("wifi_strength", Float32, queue_size = 1)
    pub_map = rospy.Publisher("wifimap", OccupancyGrid, queue_size = 1)

    rospy.Subscriber("map", OccupancyGrid, callback_map)
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        listener.waitForTransform("/map", "/base_link", now, rospy.Duration(3.0))
        trans, rot = listener.lookupTransform("/map", "/base_link", rospy.Time((0)))
        msg = create_debug_map_message(trans)

        """
        wifi_strength = get_wifi_strength()
        msg = Float32(data = wifi_strength)
        """
        if msg_map_zeros is not None:
            pub_map.publish(msg)

main()






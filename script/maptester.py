#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
from wifimap import WifiMap

if __name__=="__main__":
    rospy.init_node('debugmap_publisher')
    pub = rospy.Publisher("wifimap", OccupancyGrid, queue_size = 1)
    wfm = WifiMap()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        if wfm.map_copied is not None:
            print "hoge"
            map_msg = wfm.create_debug_map((lambda pos: 1.0))
            pub.publish(map_msg)
            r.sleep()


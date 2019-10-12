#!/usr/bin/env python
import rospy
import os 
import copy
import subprocess
import re
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
import tf
import numpy as np
import numpy.random as rn
import matplotlib.pyplot as plt

class WifiMap:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
        self.sub = rospy.Subscriber("map", OccupancyGrid, self._callback_map)
        self.map = None
        self.map_copied = None

    def _callback_map(self, msg_map):
        self.map = msg_map
        self.map_copied = copy.deepcopy(msg_map)

    def get_global_position(self):
        if self.map is None:
            return None

        now = rospy.Time.now()
        self.listener.waitForTransform("/map", "/base_link", now, rospy.Duration(3.0))
        trans, _ = self.listener.lookupTransform("/map", "/base_link", rospy.Time((0)))
        pos_map_origin = self.map.info.origin.position
        x = - pos_map_origin.x + trans[0]
        y = - pos_map_origin.y + trans[1]
        return x, y

    def create_debug_map(self):
        ret = self.get_global_position()
        if ret is None:
            return None
        x, y = ret

        if self.map_copied is None:
            return None

        info = self.map_copied.info
        resol = info.resolution
        data = [0 for i in range(len(self.map_copied.data))]
        idx_x = int(x // resol)
        idx_y = int(y // resol)
        to_idx = lambda i, j: j * info.height + i
        for i in [idx_x - 50 + i for i in range(101)]:
            for j in [idx_y - 50 + j for j in range(101)]:
                idx = to_idx(i, j)
                data[idx] = 1.0
        self.map_copied.data = data
        return self.map_copied

class DataManager():
    def __init__(self):
        self.data_x = []
        self.data_z = []
        self.n_data = 0

    def push(self, x, z):
        if self._isValidInput(x):
            self.data_x.append(x)
            self.data_z.append(z)
            self.n_data += 1

    def _isValidInput(self, x, tau = 1.0):
        if self.n_data == 0:
            return True
        x0_vec = np.array([elem[0] for elem in self.data_x])
        x1_vec = np.array([elem[1] for elem in self.data_x])

        dists = np.sqrt((x0_vec - x[0]) ** 2 + (x1_vec - x[1]) ** 2)
        dist_min = dists.min()
        isValid = (dist_min > tau)
        return isValid

def main():
    rospy.init_node('wifimap_publisher')
    pub_map = rospy.Publisher("wifimap", OccupancyGrid, queue_size = 1)
    wfm = WifiMap()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = wfm.create_debug_map()
        if msg is not None:
            pub_map.publish(msg)
        r.sleep()

#main()

if __name__=='__main__':
    dm = DataManager()
    dm.push(np.array([0, 0]), 10)

    for i in range(10000):
        x = rn.randn(2)
        dm.push(x, 0)

    x_data, y_data = [[elem[i] for elem in dm.data_x] for i in [0, 1]]
    plt.scatter(x_data, y_data)










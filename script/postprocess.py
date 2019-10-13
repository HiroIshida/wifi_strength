#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from datamanager import DataManager
from std_msgs.msg import String

from wifimap import WifiMap
from nav_msgs.msg import OccupancyGrid
import numpy as np

import json
import GPy
from math import *

def generate_gp(dm):
    kernel = GPy.kern.RBF(input_dim=2, variance=5**2 , lengthscale=1.5)
    X = np.array([[e[0], e[1]] for e in dm.data_x])
    Y = np.array([dm.data_z]).T
    model = GPy.models.GPRegression(X, Y, kernel)
    return model

def show_gp(model):
    bmin, bmax = model.get_boundary(margin = 0.2)
    N = 50
    mat_mean = np.zeros((N, N))
    mat_var = np.zeros((N, N))
    x1_lin, x2_lin = [np.linspace(bmin[i], bmax[i], N) for i in range(bmin.size)]
    X, Y = np.meshgrid(x1_lin, x2_lin)
    for i in range(N):
        for j in range(N):
            x = np.array([[x1_lin[i], x2_lin[j]]])
            mean_, var_ = model.predict(x)
            mean = mean_.item()
            var = var_.item()
            mat_mean[j, i] = mean
            mat_var[j, i] = var
            if var > 3.0:
                mat_mean[j, i] = None

    levels = [-50 + 3 * i for i in range(10)]
    fig, ax = plt.subplots() 
    cf = ax.contourf(X, Y, mat_mean, cmap = "jet") 
    fig.colorbar(cf)

    ## scatter
    x_list, y_list = [[e[i] for e in model.X.tolist()] for i in range(2)]
    plt.scatter(x_list, y_list)

class WifiCost:
    def __init__(self):
        self.pub = rospy.Publisher("wifimap", OccupancyGrid, queue_size = 1)
        self.sub = rospy.Subscriber("wifi", String, self._callback_wifi)
        self.wfm = WifiMap()

    def _callback_wifi(self, msg_wifi):
        print "hoge-1"
        dm = DataManager()

        dm = DataManager()
        dm.load("73b2room.json")
        #dm.loads(msg_wifi.data)
        model = generate_gp(dm)
        def func(pos):
            mean, var = model.predict(np.array([pos]))
            return int(-mean.item()*1.5)
        costmap = self.wfm.create_debug_map(func) 
        self.pub.publish(costmap)
        print "hoge-2"

if __name__=="__main__":
    debug = False
    if debug:
        dm = DataManager()
        dm.load("73b2room.json")
        model = generate_gp(dm)
        show_gp(model)
        plt.show()
    else:
        rospy.init_node("wifi_regression")
        wc = WifiCost()
        rospy.spin()


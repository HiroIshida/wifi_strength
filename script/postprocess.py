#!/usr/bin/env python
import rospy
import matplotlib as mpl
import matplotlib.pyplot as plt
from datamanager import DataManager
from std_msgs.msg import String

from wifimap import WifiMap
from nav_msgs.msg import OccupancyGrid
import numpy as np

import json
import gp_uncinp as gu
import math

def generate_gp(dm):
    kernel = gu.kernel.Matern23(dim = 2, l = 0.8, noise = 15)
    cov = np.zeros((2, 2))
    N = len(dm.data_x)
    X = [(e, cov) for e in dm.data_x[0:N]]
    Y = dm.data_z[0:N]
    model = gu.gp.GaussianProcess(X, Y, kernel)
    return model

def show_gp(gp):
    bmin, bmax = gp.get_boundary(margin = 0.2)
    N = 30
    mat_mean = np.zeros((N, N))
    mat_var = np.zeros((N, N))
    x1_lin, x2_lin = [np.linspace(bmin[i], bmax[i], N) for i in range(bmin.size)]
    X, Y = np.meshgrid(x1_lin, x2_lin)
    for i in range(N):
        print i
        for j in range(N):
            x = np.array([x1_lin[i], x2_lin[j]])
            mean, var = gp.predict(x)
            mat_mean[j, i] = mean
            mat_var[j, i] = var
            if var > 0.8:
                mat_mean[j, i] = None
                pass

    value_min_, value_max_ =  gp.get_value_minmax()
    value_min = math.floor(value_min_)
    value_max = math.ceil(value_max_)

    levels = [value_min + 1 * i for i in range(int(value_max - value_min + 1))]
    fig, ax = plt.subplots() 
    cf = ax.contourf(X, Y, mat_mean, cmap = "jet", levels = levels) 
    fig.colorbar(cf)

    ## scatter
    norm = mpl.colors.Normalize(vmin = value_min, vmax= value_max)
    x_list, y_list = [[e[0][i] for e in gp.X] for i in range(2)]
    plt.scatter(x_list, y_list, c = gp.Y, norm = norm, cmap = "jet", edgecolor="k")

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
    debug = True
    if debug:
        dm = DataManager()
        dm.load("73b2room.json")
        model = generate_gp(dm)
        #model.optimize()
        show_gp(model)
        plt.show()
    else:
        rospy.init_node("wifi_regression")
        wc = WifiCost()
        rospy.spin()


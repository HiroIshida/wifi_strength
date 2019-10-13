#!/usr/bin/env python
import rospy
import gpactive.utils
import matplotlib.pyplot as plt
from gpactive import *
from datamanager import DataManager
from std_msgs.msg import String

import json

class MyGP:
    def __init__(self, dm):
        kernel = rbf_uncertain_kernel(sigma = 1, l = 1.0, noise = 1e-5)
        cov = np.zeros((2, 2))
        X = [(e, cov) for e in dm.data_x]
        self.gp = GaussianProcess(X, dm.data_z, kernel)

    def show(self):
        def func(x):
            mean, var = self.gp.predict(x)
            return mean, mean
        bmin, bmax = self.gp.get_boundary(margin = 0.2)
        gpactive.utils.show2d(func, bmin, bmax)
        x_list, y_list = [[self.gp.X[i][0][j] for i in range(mygp.gp.n_train)] for j in range(2)]
        plt.scatter(x_list, y_list)
        plt.show()

global mygp
mygp = None

def callback_wifi(msg_wifi):
    print "called"
    global mygp
    dm = DataManager()
    dm.loads(msg_wifi.data)
    dm.dump("73b2.json")
    mygp = MyGP(dm)

if __name__=="__main__":
    debug = True
    if debug:
        dm = DataManager()
        dm.load("73b2.json")
        mygp = MyGP(dm)
    else:
        rospy.init_node("wifi_regression")
        rospy.Subscriber("wifi", String, callback_wifi)
        r = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            r.sleep()



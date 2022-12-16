#! /usr/bin/env python2

import matplotlib.pyplot as plt
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from matplotlib.animation import FuncAnimation
from MARS_msgs.msg import MPF_data


frame = 100
class Visualiser:
    def __init__(self):
        self.list_l, self.list_time = [], []
        self.list_bank, self.list_time = [], []
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'r-', label='UAV lateral error (meters)')
        self.ln2, = plt.plot([], [], 'b-', label='UAV bank angle (degrees)')
        plt.xlabel("time (seconds)")

    def plot_init(self):
        self.ax.set_xlim(0, 1000)
        self.ax.set_ylim(-100, 100)
        return self.ln,self.ln2

    def MPF_data_cb(self, data):
        self.mpf_data = data
        self.list_time.append(self.mpf_data.time)
        self.list_l.append(self.mpf_data.l)
        self.list_bank.append(self.mpf_data.bank)
        # self.update(frame)

    def update(self,frame):
        self.ln.set_data(self.list_time, self.list_l)
        self.ln2.set_data(self.list_time, self.list_bank)

        return self.ln, self.ln2

rospy.init_node('visual_node')
vis = Visualiser()
MPF_data_sub = rospy.Subscriber('MPF_data', MPF_data, vis.MPF_data_cb)
# ani = FuncAnimation(vis.fig,vis.update, init_func=vis.plot_init)
ani = FuncAnimation(plt.gcf(), vis.update, init_func=vis.plot_init, interval=1000)
plt.show(block=True)


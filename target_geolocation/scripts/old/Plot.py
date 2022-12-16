#! /usr/bin/env python2

import matplotlib.pyplot as plt
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from matplotlib.animation import FuncAnimation
from MARS_msgs.msg import TargetTelemetry
from geometry_msgs.msg import Point

frame = 100
class Visualiser:
    def __init__(self):
        self.list_target_y, self.list_target_x = [], []
        self.list_target_true_y, self.list_target_true_x = [], []
        self.list_UAV_x, self.list_UAV_y = [], []
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'r-', label='Target estimate position')
        self.ln2, = plt.plot([], [], 'b-', label='UAV Position')
        self.ln3, = plt.plot([], [], 'g-', label='Target true position')
        plt.xlabel("East axis")
        plt.ylabel("North axis")

    def plot_init(self):
        self.ax.set_xlim(-200, 1200)
        self.ax.set_ylim(-200, 1000)
        return self.ln,self.ln2, self.ln3

    def target_position_callback(self, data):
        self.target_position = data
        self.list_target_x.append(self.target_position.x_pos)
        self.list_target_y.append(self.target_position.y_pos)
        # self.update(frame)

    def target_true_position_callback(self, data):
        self.target_true_position = data
        self.list_target_true_x.append(self.target_true_position.x)
        self.list_target_true_y.append(self.target_true_position.y)
        # self.update(frame)

    def UAV_position_callback(self, data):
        self.UAV_position = data
        self.list_UAV_x.append(self.UAV_position.pose.position.x)
        self.list_UAV_y.append(self.UAV_position.pose.position.y)
        # self.update(frame)

    def update(self,frame):
        # self.list_target_x.append(self.target_position.x_pos)
        # self.list_target_y.append(self.target_position.y_pos)
        self.ln.set_data(self.list_target_x, self.list_target_y)

        # self.list_UAV_x.append(self.UAV_position.pose.position.x)
        # self.list_UAV_y.append(self.UAV_position.pose.position.y)
        self.ln2.set_data(self.list_UAV_x, self.list_UAV_y)

        self.ln3.set_data(self.list_target_true_x, self.list_target_true_y)

        return self.ln, self.ln2, self.ln3


rospy.init_node('lidar_visual_node')
vis = Visualiser()
target_sub = rospy.Subscriber('target_position', TargetTelemetry, vis.target_position_callback)
animated_box_sub = rospy.Subscriber('/animated_box', Point, vis.target_true_position_callback)
local_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, vis.UAV_position_callback)
ani = FuncAnimation(vis.fig,vis.update, init_func=vis.plot_init)
plt.show(block=True)

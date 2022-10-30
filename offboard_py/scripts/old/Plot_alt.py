#! /usr/bin/env python2

from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()


def animate(i):
    data = pd.read_csv('/home/joao/catkin_ws/src/plot/ficheiros_csv/target_data.csv')
    x_UAV = data['UAV_X']
    y_UAV = data['UAV_Y']
    x_target = data['target.x_pos']
    y_target = data['target.y_pos']
    x_true = data['target_x_true']
    y_true = data['target_y_true']

    plt.cla()

    plt.plot(x_UAV, y_UAV, label='UAV position')
    plt.plot(x_target, y_target, label='Target position')
    plt.plot(x_true, y_true, label='True target position')

    plt.legend(loc='upper left')
    plt.tight_layout()


ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()



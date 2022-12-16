#! /usr/bin/env python2
import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use('fivethirtyeight')

x_vals = []
y_vals = []

index = count()


def animate(i):
    data = pd.read_csv('/home/joao/catkin_ws/src/plot/ficheiros_csv/validation_MPF.csv')
    x = data['time']
    y1 = data['erro l']
    y2 = data['bank']

    plt.cla()

    plt.plot(x, y1, label='erro lateral')
    plt.plot(x, y2, label='bank')

    plt.legend(loc='upper left')
    plt.tight_layout()


ani = FuncAnimation(plt.gcf(), animate, interval=1000)

plt.tight_layout()
plt.show()
import csv
import matplotlib
matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})
import matplotlib.pyplot as plt
import numpy as np
import os
import math
from mpl_toolkits import mplot3d

number_of_uavs = 3

# Plots Target and Target Estimate
def plot_paths2d(snap_time):

    plt.figure(figsize=(8, 6))
    i = 0
    snap_time = snap_time*20
    #ax = plt.subplots()

    while i < number_of_uavs:
        file_with_data = open('data' + str(i) + '.csv')
        csvreader = csv.reader(file_with_data)
        target_x = []
        target_y = []
        pos_x = []
        pos_y = []

        for row in csvreader:
            pos_x.append(float(row[7]))
            pos_y.append(float(row[6]))
            target_x.append(float(row[9]))
            target_y.append(float(row[10]))
        



        if i == 0:
            #circle1 = plt.Circle((target_y[snap_time],target_x[snap_time]), 200, color='k', '--')
            #plt.gca().add_patch(circle1)

            # theta goes from 0 to 2pi
            theta = np.linspace(0, 2*np.pi, 100)

            # the radius of the circle
            r = 200

            # compute x1 and x2
            x1 = r*np.cos(theta) + target_y[snap_time]
            x2 = r*np.sin(theta) + target_x[snap_time]
            plt.plot(x1, x2, 'k--')

            plt.gca().set_aspect('equal')

            plt.plot(target_y[0:snap_time],target_x[0:snap_time], 'k')
            plt.plot(target_y[0],target_x[0], 'k*')
            plt.plot(target_y[snap_time],target_x[snap_time], 'ko')

            plt.plot(pos_y[0], pos_y[0], 'r*')
            plt.plot(pos_y[snap_time], pos_x[snap_time], 'ro')

            plt.plot(pos_y[0:snap_time], pos_x[0:snap_time], 'r')
        elif i == 1:
            plt.plot(pos_y[0], pos_y[0], 'g*')
            plt.plot(pos_y[snap_time], pos_x[snap_time], 'go')

            plt.plot(pos_y[0:snap_time], pos_x[0:snap_time], 'g')
        elif i == 2:
            plt.plot(pos_y[0], pos_y[0], 'b*')
            plt.plot(pos_y[snap_time], pos_x[snap_time], 'bo')

            plt.plot(pos_y[0:snap_time], pos_x[0:snap_time], 'b')
        
        i = i + 1

    plt.ylabel("Norh (m)")
    plt.xlabel("East (m)")
    plt.grid()
    plt.savefig('Graphs/snap' + str(int(snap_time/20)) + '.pgf')

plot_paths2d(75) #Seconds of snapshot
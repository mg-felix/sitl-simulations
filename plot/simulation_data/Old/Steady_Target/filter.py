import csv
import matplotlib
"""matplotlib.use("pgf")
matplotlib.rcParams.update({
    "pgf.texsystem": "pdflatex",
    'font.family': 'serif',
    'text.usetex': True,
    'pgf.rcfonts': False,
})"""
import matplotlib.pyplot as plt
import numpy as np
import os
import math
from mpl_toolkits import mplot3d
from scipy.signal import butter,filtfilt

number_of_uavs = 3


def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff/nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

# Plots commanded roll
i = 0
plt.figure(1)
while i < number_of_uavs:
    file_with_data = open('data'+ str(i) +'.csv')

    csvreader = csv.reader(file_with_data)

    roll_cmd = []
    time_passed = []

    for row in csvreader:
        roll_cmd.append(180/math.pi * float(row[3]))
        time_passed.append(float(row[2]))

    # Filter RequiremParameters
    fs = 20 # Sample rate, Hz
    cutoff = 0.1 # Cutoff Frequency, Hz
    nyq = 0.5*fs # Nyquist frequency
    order = 1 # Sin wave can be approx by a quadratic
    
    #roll_cmd = butter_lowpass_filter(roll_cmd, cutoff, nyq, order)
    
    
    if i == 0:
        plt.plot(time_passed, roll_cmd, 'r')
    elif i == 1:
        plt.plot(time_passed, roll_cmd, 'g')
    elif i == 2:
        plt.plot(time_passed, roll_cmd, 'b')
    
    i = i + 1


    


plt.ylabel("Commanded roll (degrees)")
plt.xlabel("Time (s)")
plt.grid()
#plt.savefig('Graphs/roll_cmd.pgf')

plt.show()
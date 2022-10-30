import csv
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import os
import math

#number_of_uavs = os.system.argv[1]

number_of_uavs = 3

# Check if it's a valid number
if math.isnan(number_of_uavs):
    print("Input not a number!")
    exit()
elif number_of_uavs > 10 or number_of_uavs < 1:
    print("Please choose a number between 1 and 10")
    exit()

# Plots e1
i = 0
plt.figure(1)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	e_x = []
	time_passed = []

	for row in csvreader:
		e_x.append(float(row[0]))
		time_passed.append(float(row[2]))
	
	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, e_x)
	if i == ((number_of_uavs - 1 )/ 2):
		plt.ylabel("e1 (m)")
	plt.xlabel("Time (s)")
	plt.grid()
	
	i = i + 1
# Plots e2
i = 0
plt.figure(2)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	e_y = []
	time_passed = []

	for row in csvreader:
		e_y.append(float(row[1]))
		time_passed.append(float(row[2]))
	
	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, e_y)
	if i == ((number_of_uavs - 1 )/ 2):
		plt.ylabel("e2 (m)")
	plt.xlabel("Time (s)")
	plt.grid()	
	i = i + 1
# Plots commanded velocity
i = 0
plt.figure(3)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	v_cmd = []
	time_passed = []

	for row in csvreader:
		v_cmd.append(float(row[4]))
		time_passed.append(float(row[2]))
	
	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, v_cmd)
	plt.grid()
	#plt.tick_params(labelcolor='none', which='both', top=False, bottom=False, left=False, right=False)
	if i == ((number_of_uavs - 1 )/ 2):
		plt.ylabel("Commanded velocity (m/s)")

	plt.xlabel("Time (s)")

	i = i + 1

# Plots commanded roll
i = 0
plt.figure(4)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	roll_cmd = []
	time_passed = []

	for row in csvreader:
		roll_cmd.append(float(row[3])*180/math.pi)
		time_passed.append(float(row[2]))
	
	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, roll_cmd)
	if i == ((number_of_uavs - 1 )/ 2):
		plt.ylabel("Commanded roll (degrees)")
	plt.xlabel("Time (s)")
	plt.grid()
	
	i = i + 1


# Plots gamma
i = 0
plt.figure(5)
while i < 3:#number_of_uavs:
	file_with_data = open('particle_data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	gamma = []
	time_passed = []

	for row in csvreader:
		gamma.append(float(row[0]))
		time_passed.append(float(row[2]))
	plt.plot(time_passed, gamma)
	plt.ylabel("Gamma (m)")
	plt.xlabel("Time (s)")
	plt.grid()
	i = i + 1


# Plots gamma_dot
i = 0
plt.figure(6)
while i < number_of_uavs:
	file_with_data = open('particle_data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	gamma_dot = []
	time_passed = []

	for row in csvreader:
		gamma_dot.append(float(row[1]))
		time_passed.append(float(row[2]))

	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, gamma_dot)
	if i ==  ((number_of_uavs - 1 )/ 2):
		plt.ylabel("Particle velocity in the path (m/s)")
	plt.xlabel("Time (s)")
	plt.grid()
	
	i = i + 1

# Plots gamma_tilde
i = 0
plt.figure(7)
while i < number_of_uavs:
	file_with_data = open('particle_data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	gamma_tilde = []
	time_passed = []

	for row in csvreader:
		gamma_tilde.append(float(row[3]) + 2*math.pi*200/3)
		time_passed.append(float(row[2]))

	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, gamma_tilde)
	if i ==  ((number_of_uavs - 1 )/ 2):
		plt.ylabel("Arc distance between particles (m)")
	plt.xlabel("Time (s)")

	
	i = i + 1
	plt.grid()

# Plots gamma_dot_minus_1
"""
i = 0
plt.figure(8)
while i < number_of_uavs:
	file_with_data = open('particle_data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	gamma_dot_minus_1 = []
	time_passed = []

	for row in csvreader:
		gamma_dot_minus_1.append(float(row[4]))
		time_passed.append(float(row[2]))

	plt.plot(time_passed, gamma_dot_minus_1)
	plt.ylabel("Particle -1 velocity in the path (m/s)")
	plt.xlabel("Time (s)")
	
	i = i + 1
"""
# Plots Altitude
i = 0
plt.figure(9)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	altitude = []
	time_passed = []

	for row in csvreader:
		altitude.append(float(row[5]))
		time_passed.append(float(row[2]))

	plt.subplot(int(str(number_of_uavs) + str(1) + str(i+1)))
	plt.plot(time_passed, altitude)
	if i == ((number_of_uavs - 1 )/ 2):
		plt.ylabel("Altitude (m)")
	plt.xlabel("Time (s)")
	plt.grid()
	
	i = i + 1

# Plots Target and Target Estimate
plt.figure(9)
def plot_target():
	file_with_data = open('target.csv')
	csvreader = csv.reader(file_with_data)
	target_x = []
	target_y = []

	for row in csvreader:
		target_x.append(float(row[10]))
		target_y.append(float(row[11]))
		target_x_estimate.append(float(row[2]))
		target_y_estimate.append(float(row[3]))

	plt.plot(target_x, target_y, 'b*')

	plt.plot(target_x_estimate, target_y_estimate, 'bo')

	plt.ylabel("Norht (m)")
	plt.xlabel("East (m)")
	plt.grid()

plt.show()

file_with_data.close()
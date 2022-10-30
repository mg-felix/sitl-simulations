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
	
	if i == 0:
		plt.plot(time_passed, e_x, 'r')
	elif i == 1:
		plt.plot(time_passed, e_x, 'g')
	elif i == 2:
		plt.plot(time_passed, e_x, 'b')
	
	i = i + 1

plt.ylabel("e1 (m)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/err1.pgf')
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
	
	if i == 0:
		plt.plot(time_passed, e_y, 'r')
	elif i == 1:
		plt.plot(time_passed, e_y, 'g')
	elif i == 2:
		plt.plot(time_passed, e_y, 'b')
	
	i = i + 1

plt.ylabel("e2 (m)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/err2.pgf')

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
	
	if i == 0:
		plt.plot(time_passed, v_cmd, 'r')
	elif i == 1:
		plt.plot(time_passed, v_cmd, 'g')
	elif i == 2:
		plt.plot(time_passed, v_cmd, 'b')

	
	
	i = i + 1

plt.grid()
plt.ylabel("Commanded velocity (m/s)")
plt.xlabel("Time (s)")
plt.savefig('Graphs/vel_cmd.pgf')

# Plots commanded roll
i = 0
plt.figure(4)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	roll_cmd = []
	time_passed = []

	for row in csvreader:
		roll_cmd.append(180/math.pi * float(row[3]))
		time_passed.append(float(row[2]))
	
	
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
plt.savefig('Graphs/roll_cmd.pgf')
"""
# Plots gamma
i = 0
plt.figure(5)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	gamma = []
	time_passed = []

	for row in csvreader:
		gamma.append(float(row[6]))
		time_passed.append(float(row[2]))
	plt.plot(time_passed, gamma[0])
	i = i + 1
"""

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

	if i == 0:
		plt.plot(time_passed, gamma_dot, 'r')
	elif i == 1:
		plt.plot(time_passed, gamma_dot, 'g')
	elif i == 2:
		plt.plot(time_passed, gamma_dot, 'b')
	
	i = i + 1

plt.ylabel("Particle velocity in the path (m/s)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/vel_part.pgf')

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

	if i == 0:
		plt.plot(time_passed, gamma_tilde, 'r')
	elif i == 1:
		plt.plot(time_passed, gamma_tilde, 'g')
	elif i == 2:
		plt.plot(time_passed, gamma_tilde, 'b')
	
	i = i + 1

plt.ylabel("Arc distance between particles (m)")
plt.grid()
plt.xlabel("Time (s)")
plt.savefig('Graphs/dist_part.pgf')

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

	if i == 0:
		plt.plot(time_passed, altitude, 'r')
	elif i == 1:
		plt.plot(time_passed, altitude, 'g')
	elif i == 2:
		plt.plot(time_passed, altitude, 'b')
	
	i = i + 1

plt.ylabel("Altitude (m)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/altitude.pgf')

# Plots Target and Target Estimate
"""
plt.figure(10)
def plot_target():
	file_with_data = open('target.csv')
	csvreader = csv.reader(file_with_data)
	target_x = []
	target_y = []
	target_x_estimate = []
	target_y_estimate = []

	for row in csvreader:
		target_x_estimate.append(float(row[2]))
		target_y_estimate.append(float(row[3]))
		target_x.append(float(row[10]))
		target_y.append(float(row[11]))

	plt.plot(target_x, target_y, 'k*')

	#plt.plot(target_x_estimate, target_y_estimate, 'ko')

	plt.ylabel("Norht (m)")
	plt.xlabel("East (m)")
	plt.grid()
	plt.savefig('Graphs/target.pgf')
"""
	
# Plots Target and Target Estimate
def plot_paths2d():
	plt.figure(15)
	i = 0

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
		
		plt.plot(target_y,target_x, 'k')
		plt.plot(target_y[0],target_x[0], 'k*')
		plt.plot(target_y[-1],target_x[-1], 'ko')

		target_real_x = [0,50,150,175,250,150,0]
		target_real_y = [0,0,0,0,0,0,0]
		plt.plot(target_real_x, target_real_y,'m+-')

		if i == 0:
			plt.plot(pos_y[0], pos_x[0], 'r*')
			plt.plot(pos_y[-1], pos_x[-1], 'ro')

			plt.plot(target_y,target_x, 'k')
			plt.plot(pos_y, pos_x, 'r')
		elif i == 1:
			plt.plot(pos_y[0], pos_x[0], 'g*')
			plt.plot(pos_y[-1], pos_x[-1], 'go')

			plt.plot(pos_y, pos_x, 'g')
		elif i == 2:
			plt.plot(pos_y[0], pos_x[0], 'b*')
			plt.plot(pos_y[-1], pos_x[-1], 'bo')

			plt.plot(pos_y, pos_x, 'b')
		
		i = i + 1

	plt.ylabel("Norh (m)")
	plt.xlabel("East (m)")
	plt.grid()
	plt.savefig('Graphs/NEplane.pgf')

# Plots Target and Target Estimate

def plot_target():
	plt.figure(10)
	file_with_data = open('target.csv')
	csvreader = csv.reader(file_with_data)
	target_x = []
	target_y = []
	target_x_estimate = []
	target_y_estimate = []

	for row in csvreader:
		target_x_estimate.append(float(row[2]))
		target_y_estimate.append(float(row[3]))
		target_x.append(float(row[10]))
		target_y.append(float(row[11]))

	plt.plot(target_x, target_y, 'k*')

	#plt.plot(target_x_estimate, target_y_estimate, 'ko')

	plt.ylabel("Norht (m)")
	plt.xlabel("East (m)")
	plt.grid()
	plt.savefig('Graphs/target.pgf')

	
# Plots UAV and target paths
def plot_paths3d():

	plt.figure(11)
	ax = plt.axes(projection='3d')
	i = 0

	while i < number_of_uavs:
		file_with_data = open('data' + str(i) + '.csv')
		csvreader = csv.reader(file_with_data)
		target_x = []
		target_y = []
		target_z = []
		pos_x = []
		pos_y = []
		pos_z = []

		for row in csvreader:
			pos_x.append(float(row[6]))
			pos_y.append(float(row[7]))
			pos_z.append(float(row[8]))
			target_x.append(float(row[9]))
			target_y.append(float(row[10]))
			target_z.append(float(row[11]))
		
		target_real_x = [0,50,150,175,250,150,0]
		target_real_y = [0,0,0,0,0,0,0]
		target_real_z = [0,0,0,0,0,0,0]
		ax.plot3D(target_real_x, target_real_y, target_real_z, 'm+-')

		if i == 0:
			ax.plot3D(pos_x, pos_y, pos_z, 'r')
			ax.plot3D(target_y, target_x, target_z, 'k')
			ax.plot3D(target_y[0], target_x[0], target_z[0], 'k*')
			ax.plot3D(target_y[-1], target_x[-1], target_z[-1], 'ko')
			ax.plot3D(pos_x[0], pos_y[0], pos_z[0], 'r*')
			ax.plot3D(pos_x[-1], pos_y[-1], pos_z[-1], 'ro')
		elif i == 1:
			ax.plot3D(pos_x, pos_y, pos_z, 'g')
			ax.plot3D(pos_x[0], pos_y[0], pos_z[0], 'g*')
			ax.plot3D(pos_x[-1], pos_y[-1], pos_z[-1], 'go')
		elif i == 2:
			ax.plot3D(pos_x, pos_y, pos_z, 'b')
			ax.plot3D(pos_x[0], pos_y[0], pos_z[0], 'b*')
			ax.plot3D(pos_x[-1], pos_y[-1], pos_z[-1], 'bo')
		
		i = i + 1

	ax.set_ylabel("North (m)")
	ax.set_xlabel("East (m)")
	ax.set_zlabel("Altitude (m)")
	#plt.grid()
	plt.savefig('Graphs/paths3D.pgf')


# Plots commanded heading rate
i = 0
plt.figure(12)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	psi_dot_cmd = []
	time_passed = []

	for row in csvreader:
		psi_dot_cmd.append(180/math.pi * float(row[14]))
		time_passed.append(float(row[2]))
	
	
	if i == 0:
		plt.plot(time_passed, psi_dot_cmd, 'r')
	elif i == 1:
		plt.plot(time_passed, psi_dot_cmd, 'g')
	elif i == 2:
		plt.plot(time_passed, psi_dot_cmd, 'b')
	
	i = i + 1
	
plt.ylabel("Commanded heading rate (degrees/s)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/psi_dot_cmd.pgf')

# Plots commanded pitch
i = 0
plt.figure(13)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	pitch_cmd = []
	time_passed = []

	for row in csvreader:
		pitch_cmd.append(180/math.pi * float(row[13]))
		time_passed.append(float(row[2]))
	
	
	if i == 0:
		plt.plot(time_passed, pitch_cmd, 'r')
	elif i == 1:
		plt.plot(time_passed, pitch_cmd, 'g')
	elif i == 2:
		plt.plot(time_passed, pitch_cmd, 'b')
	
	i = i + 1
	
plt.ylabel("Commanded pitch (degrees)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/pitch_cmd.pgf')

# Plots commanded thrust
i = 0
plt.figure(14)
while i < number_of_uavs:
	file_with_data = open('data'+ str(i) +'.csv')

	csvreader = csv.reader(file_with_data)

	thrust_cmd = []
	time_passed = []

	for row in csvreader:
		thrust_cmd.append(float(row[12]))
		time_passed.append(float(row[2]))
	
	
	if i == 0:
		plt.plot(time_passed, thrust_cmd, 'r')
	elif i == 1:
		plt.plot(time_passed, thrust_cmd, 'g')
	elif i == 2:
		plt.plot(time_passed, thrust_cmd, 'b')
	
	i = i + 1
	
plt.ylabel("Commanded thrust (normalized)")
plt.xlabel("Time (s)")
plt.grid()
plt.savefig('Graphs/thrust_cmd.pgf')

#plot_target()
plot_paths2d()
plot_paths3d()

plt.show()

file_with_data.close()
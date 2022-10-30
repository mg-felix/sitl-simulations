#! /usr/bin/env python3

from __future__ import division

import numpy as np
import rospy
import math

global integral_e_previous
integral_e_previous = 0.0

def roll_calculation(UAV_position, VFR, target, particle, delta_t):

	global integral_e_previous

	# Target position and velocity
	target_vel = [target.vy, target.vx]
	x_ref = target.y_pos
	y_ref = target.x_pos

	# UAV Position
	x_actual = UAV_position.pose.position.y
	y_actual = UAV_position.pose.position.x

	# UAV Current Heading
	psi_actual = (VFR.heading * np.pi)/180

	# CONTROL LAW ######

	# Constants definitions

	Kp = np.array([[9e-2, 0],
	[0, 5e-3]])

	eps1 = 10
	eps2 = 0
	eps = np.array([eps1, eps2])
	Delta = np.array([[1, -eps2], 
	[0, eps1]])
	Delta_mp_pseudo = np.linalg.inv(Delta)

	# Rotation matrix and aircraft position
	current_pos = np.array([x_actual, y_actual])

	#psi_actual = 0 # ELIMINAR (TESTE)

	R_R_I = np.array([[math.cos(psi_actual), -math.sin(psi_actual)], [math.sin(psi_actual), math.cos(psi_actual)]])

	# Path and particle in path related definitions
	radius = 200 # 100 to camera simulations and 200 for SITL validation

	gamma = particle.gamma
	vd = 15
	dpdgamma = np.array([-math.sin(gamma/radius), math.cos(gamma/radius)])
	particle_pos = np.array([radius*math.cos(gamma/radius) + x_ref, radius*math.sin(gamma/radius) + y_ref])

	# Error
	e = np.array(np.dot(np.transpose(R_R_I), (current_pos - particle_pos)) + eps)

	# Calculating the control input
	u = np.array(np.dot(Delta_mp_pseudo, (np.dot(-Kp, e)) + np.dot(np.transpose(R_R_I), target_vel) + np.dot(np.transpose(R_R_I), dpdgamma*vd)))

	# Anti wind-up
	integral_e = integral_e_previous + (e * delta_t)
	if integral_e[0] > 0.5:
		integral_e[0] = 0.5
	elif integral_e[0] < -0.5:
		integral_e[0] = -0.5
	
	if integral_e[1] > 0.03:
		integral_e[1] = 0.03
	elif integral_e[1] < -0.03:
		integral_e[1] = -0.03

	integral_e_previous = integral_e
	integral_e[0] = 0
	integral_e[1] = 0

	v = u[0] + integral_e[0]
	psi_dot = u[1] + integral_e[1]

	v_min = 10
	v_max = 20

	if v > v_max:
		v = v_max
	elif v < v_min:
		v = v_min

	roll = math.atan(psi_dot * VFR.airspeed / 9.80665)

	# Limiting the control commands values
	if abs(roll) > 0.4311:
		roll = np.sign(roll) * 0.4311

	return roll, e, v, psi_dot

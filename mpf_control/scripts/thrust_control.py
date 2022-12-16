#! /usr/bin/env python3

from __future__ import division

import numpy
import numpy as np
import rospy
import math

global integral_previous
integral_previous = 0.0

def thrust_input(vfr_hud, delta_t, v):

	global integral_previous

	velocity_setpoint = v
	velocity_error = velocity_setpoint - vfr_hud.airspeed
	integral = integral_previous + (0.3 * velocity_error * delta_t)

	# Integral should be between -0.2 and 0.2

	if integral < -0.2:
		integral = -0.2
	elif integral > 0.2:
		integral = 0.2
	else:
		integral

	integral_previous = integral

	thrust = integral + 0.2 * velocity_error + 0.5

	# Power interval between 0 and 1

	if thrust < 0.05:
		thrust = 0.05
	elif thrust > 1.0:
		thrust = 1.0
	else:
		thrust

	return thrust

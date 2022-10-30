#! /usr/bin/env python3

from __future__ import division

import numpy
import numpy as np
import rospy
import math

global integral_pitch_previous
integral_pitch_previous = 0.0

def compute_pitch(altitude, delta_t):

	global integral_pitch_previous

	altitude_setpoint = 30

	# Altitude error calculation
	altitude_error = altitude_setpoint - altitude.local

	# Integral Calculation
	integral_pitch = integral_pitch_previous + (0.2 * altitude_error * delta_t)

	# Integral should be between -0.03 and 0.03
	if integral_pitch < -0.03:
		integral_pitch = -0.03
	elif integral_pitch > 0.03:
		integral_pitch = 0.03
	else:
		integral_pitch

	integral_pitch_previous = integral_pitch

	# Pitch final value calculation
	pitch = integral_pitch + 0.2 * altitude_error

	# Pitch interval between -0.2 and 0.2
	if pitch < -0.2:
		pitch = -0.2
	elif pitch > 0.2:
		pitch = 0.2
	else:
		pitch

	return pitch

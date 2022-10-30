#! /usr/bin/env python3

import rospy
import math
import pickle
import os.path
import csv

from MARS_msgs.msg import TargetTelemetry

def target_node():

	# Creates the publisher to publish the target position on the topic "target_position"
	pub = rospy.Publisher('target_position', TargetTelemetry, queue_size=10)
	rospy.init_node('target_node', anonymous=True)

	# Publishing rate
	rate = rospy.Rate(20)

	msg = TargetTelemetry()

	# Time related variables initialization
	start_time = rospy.get_time()
	curr_time = rospy.get_time()
	timer = 0.0
	dt = 0.0

	# Target variables initialization (All changeble)
	target_accel = 0.0
	position_x = 0.0
	position_y = 0.0
	target_psi = 0 #math.pi/4
	target_velocity = 0

	if os.path.isfile("/home/mgfelix/catkin_ws/src/plot/simulation_data/target.csv"):
		os.remove("/home/mgfelix/catkin_ws/src/plot/simulation_data/target.csv")

	while not rospy.is_shutdown():

		# Time variation since last call on loop
		dt = rospy.get_time() - curr_time

		# Total time counter
		timer = rospy.get_time() - start_time

		curr_time = rospy.get_time()

		# Target movement calculations
		target_omega = 0#.02*math.cos(0.03*curr_time) # Changeble
		target_accel = 0#.1*math.sin(0.07*curr_time) # Changeble

		target_psi = target_psi + target_omega * dt
		target_velocity = target_velocity + (target_accel * dt) 
		target_vx = target_velocity * math.cos(target_psi)
		target_vy = target_velocity * math.sin(target_psi)
		position_x = position_x + target_vx * dt
		position_y = position_y + target_vy * dt

		# Defines message to be sent with target movement variables
		msg.x_pos = position_x
		msg.y_pos = position_y
		msg.vx = target_vx
		msg.vy = target_vy
		msg.omega = target_omega
		msg.accel = target_accel
		msg.vel = target_velocity
		msg.psi = target_psi

		pub.publish(msg)

		# Publishes the target data
		"""
		if (rospy.get_time()<100):
			pub.publish(msg)
		elif(rospy.get_time()>100):
			rospy.loginfo_once("Stopped publishing in TargetUpdated.")"""

		d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/target_real.csv", "a")
		with d:
			writer = csv.writer(d)
			writer.writerow([position_x,
							position_y,
							])


		# Keeps the rate fixed
		rate.sleep()

if __name__ == '__main__':
	try:
		target_node()
	except rospy.ROSInterruptException:
		pass

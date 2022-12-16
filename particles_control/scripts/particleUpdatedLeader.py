#! /usr/bin/env python3

from __future__ import division
from cmath import pi
import rospy
import math

from particles_msgs.msg import ParticleTelemetry

def particle_node():

	# Initiates the node
	rospy.init_node('particle_node', anonymous=True)

	# Defines the publishers needed to publish updated leader particle position
	pub = rospy.Publisher('uav0/particle_position', ParticleTelemetry, queue_size = 10)

	rate = rospy.Rate(20)

	# Defines the type of message to be sent 
	msg = ParticleTelemetry()

	# Leader Particle Properties
	leader_velocity = 15 # m/s, velocity of particle along the path
	gamma_leader = 0 # Particle initial position

	# Time related variables
	start_time = rospy.get_time()
	curr_time = rospy.get_time()
	dt = 0.0

	while not rospy.is_shutdown():

		# Updates delta t every iteration
		dt = rospy.get_time() - curr_time
		curr_time = rospy.get_time()

		gamma_dot_leader = leader_velocity
		gamma_leader = gamma_leader + gamma_dot_leader*dt

		# Defining the messages
		msg.gamma = gamma_leader
		msg.gamma_dot = gamma_dot_leader

		# Publishing the message
		pub.publish(msg)

		rate.sleep()


if __name__ == '__main__':
	try:
		particle_node()
	except rospy.ROSInterruptException:
		pass

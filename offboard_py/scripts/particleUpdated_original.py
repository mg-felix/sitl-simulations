#! /usr/bin/env python2

from __future__ import division
from cmath import pi
import rospy
import math

from MARS_msgs.msg import ParticleTelemetry

def particle_cb0(data):
    global gamma0, gamma_dot_0
    gamma0 = data.gamma
    gamma_dot_0 = data.gamma_dot

def particle_cb1(data):
    global gamma1, gamma_dot_1
    gamma1 = data.gamma
    gamma_dot_1 = data.gamma_dot

def particle_cb2(data):
    global gamma2, gamma_dot_2
    gamma2 = data.gamma
    gamma_dot_2 = data.gamma_dot

def particle_node():

    # Defines the publishers needed to publish updated particle positions
    pub0 = rospy.Publisher('uav0/particle_position', ParticleTelemetry, queue_size = 10)
    pub1 = rospy.Publisher('uav1/particle_position', ParticleTelemetry, queue_size = 10)
    pub2 = rospy.Publisher('uav2/particle_position', ParticleTelemetry, queue_size = 10)

    sub0 = rospy.Subscriber('uav0/particle_position', ParticleTelemetry, particle_cb0)
    sub1 = rospy.Subscriber('uav0/particle_position', ParticleTelemetry, particle_cb1)
    sub2 = rospy.Subscriber('uav0/particle_position', ParticleTelemetry, particle_cb2)
    
    rospy.init_node('particle_node', anonymous=True)

    rate = rospy.Rate(20)

    msg0 = ParticleTelemetry()
    msg1 = ParticleTelemetry()
    msg2 = ParticleTelemetry()

    # Path Definition
    radius = 200

    # Leader Particle Properties
    leader_velocity = 12  # m/s, velocity of particle along the path
    delta_gamma = 2*math.pi*radius/3  # Distance between particles
    gamma_leader = 0

    # Initialization of variables
    gamma_0 = 0
    gamma_1 = 0
    gamma_2 = 0

    # Control gains

    Ku = 0.1
    beta = 50

    # Time related variables
    start_time = rospy.get_time()
    curr_time = rospy.get_time()
    timer = 0.0
    dt = 0.0

    while not rospy.is_shutdown():

        global gamma0, gamma1, gamma2, gamma_dot_0, gamma_dot_1, gamma_dot_2

        dt = rospy.get_time() - curr_time
        timer = rospy.get_time() - start_time
        curr_time = rospy.get_time()

        gamma_dot_leader = leader_velocity
        gamma_leader = gamma_leader + gamma_dot_leader*dt
        
        # Particles to be followed
        gamma_tilde_0 = gamma_0 - gamma_leader + delta_gamma
        gamma_tilde_1 = gamma_1 - gamma_0 + delta_gamma 
        gamma_tilde_2 = gamma_2 - gamma_1 + delta_gamma       

        gamma_dot_0 = gamma_dot_leader - beta*math.tanh(Ku*gamma_tilde_0)
        gamma_dot_1 = gamma_dot_0 - beta*math.tanh(Ku*gamma_tilde_1)
        gamma_dot_2 = gamma_dot_1 - beta*math.tanh(Ku*gamma_tilde_2)

        gamma_0 = gamma_0 + gamma_dot_0*dt
        gamma_1 = gamma_1 + gamma_dot_1*dt
        gamma_2 = gamma_2 + gamma_dot_2*dt

        # Defining the messages
        msg0.gamma = gamma_0
        msg0.gamma_dot = gamma_dot_0

        msg1.gamma = gamma_1
        msg1.gamma_dot = gamma_dot_1

        msg2.gamma = gamma_2
        msg2.gamma_dot = gamma_dot_2

        # Publishing the message
        pub0.publish(msg0)
        pub1.publish(msg1)
        pub2.publish(msg2)
    
        rate.sleep()


if __name__ == '__main__':
    try:
        particle_node()
    except rospy.ROSInterruptException:
        pass

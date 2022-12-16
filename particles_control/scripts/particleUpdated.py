#! /usr/bin/env python3

from __future__ import division
from cmath import pi
import rospy
import math
import csv
import os

from particles_msgs.msg import ParticleTelemetry

#global gamma_minus_1, gamma_dot_minus_1

def particle_cb(data):
    global gamma_minus_1, gamma_dot_minus_1
    gamma_minus_1 = data.gamma
    gamma_dot_minus_1 = data.gamma_dot

def particle_node():

    rospy.init_node('particle_node', anonymous=True)
    
    global gamma_minus_1, gamma_dot_minus_1
    

    # Gets the uav_id from launch file
    uav_id = rospy.get_param("~uav_id")
    uav_id_number = uav_id[-1]

    # Defines the publishers needed to publish updated particle positions
    pub = rospy.Publisher('uav' + str(int(uav_id_number) + 1) + '/particle_position', ParticleTelemetry, queue_size = 10)

    sub = rospy.Subscriber(uav_id + '/particle_position', ParticleTelemetry, particle_cb)

    N = 3 # Can be changed to get the value from launch file

    rate = rospy.Rate(20)

    msg = ParticleTelemetry()

    # Path Definition
    radius = 200 # 100 to camera simulations and 200 for SITL validation

    # Distance between particles
    delta_gamma = 2*math.pi*radius/N  # Distance between particles

    # Initialization of variables
    gamma = -(int(uav_id_number) + 1) * delta_gamma # Particle Initial Position
    gamma_minus_1 = -(int(uav_id_number)) * delta_gamma
    gamma_dot_minus_1 = 15

    # Control gains

    Ku = 0.1
    beta = 0.5

    # Time related variables
    start_time = rospy.get_time()
    curr_time = rospy.get_time()
    timer = 0.0
    dt = 0.0

    if os.path.isfile("/home/mgfelix/catkin_ws/src/plot/simulation_data/particle_data" + uav_id_number + ".csv"):
        os.remove("/home/mgfelix/catkin_ws/src/plot/simulation_data/particle_data" + uav_id_number + ".csv")

    while not rospy.is_shutdown():

        dt = rospy.get_time() - curr_time
        timer = rospy.get_time() - start_time
        curr_time = rospy.get_time()

        # Particle to be followed
        gamma_tilde = gamma - gamma_minus_1 + delta_gamma

        gamma_dot = gamma_dot_minus_1 - beta*math.tanh(Ku*gamma_tilde)

        gamma = gamma + gamma_dot*dt

        # Defining the message
        msg.gamma = gamma
        msg.gamma_dot = gamma_dot


        # Publishing the message
        pub.publish(msg)
        f = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/particle_data" + uav_id_number + ".csv", "a")
        with f:
            writer = csv.writer(f)
            writer.writerow([
                gamma,
                gamma_dot,
                timer,
                gamma_tilde,
                gamma_dot_minus_1,
                gamma_minus_1
            ])

        rate.sleep()


if __name__ == '__main__':
    try:
        particle_node()
    except rospy.ROSInterruptException:
        pass

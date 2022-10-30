#! /usr/bin/env python3

# Import libraries
import rospy
import math
import csv
import time
import os

# Import messages
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3
from mavros_msgs.msg import State, Altitude, HomePosition, ExtendedState, PositionTarget, AttitudeTarget, VFR_HUD, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode, StreamRateRequest, StreamRate
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Import custom messages
from MARS_msgs.msg import TargetTelemetry # Message to send the target movement data
from MARS_msgs.msg import MPF_data # Message to save the simulation data
from MARS_msgs.msg import ParticleTelemetry # Message to send the gammas and gamma dots
# from sensor_msgs.msg import Imu

# Import custom scripts
import roll_control
import pitch_control
import thrust_control

# Function that starts and keeps offboard control
def start_offboard_control():
	while not rospy.is_shutdown() and current_state.mode == "OFFBOARD" and current_state.armed:

		# Gets current and elapsed time in every loop iteration
		curr_time = rospy.get_time()
		time_passed = curr_time - start_time

		# Calls function to compute time related variables
		compute_time()

		# Calls functions to send attitude
		send_att()

		# Saves Data
		MPF_Data.time = time_passed
		MPF_Data.l = e
		MPF_Data.bank = bank
		MPF_data_pub.publish(MPF_Data)

		#print(curr_time)

		# Saves simulation data into .csv file
		f = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/data" + uav_id_number + ".csv", "a")
		with f:
			writer = csv.writer(f)
			writer.writerow([  
				e[0],
				e[1],
				curr_time,
				roll,
				v_cmd,
				altitude.local,
				local_position.pose.position.y,
				local_position.pose.position.x,
				local_position.pose.position.z,
				target.x_pos,
				target.y_pos,
				target.z_pos,
				thrust,
				pitch,
				heading_rate_cmd
			])
		rate.sleep()
            
# Computes how much time has passed since last iteration
def compute_time():
	global previous_time, delta_t
	delta_t = rospy.get_time() - previous_time
	previous_time = rospy.get_time()

# Sets up all the needed services
def setup_services(uav_id):

	# Service timeout value - after how long should stop trying setup/call the services
	service_timeout = 30

	# Waits for the services to be available for each UAV
	rospy.wait_for_service(uav_id + "/mavros/cmd/arming", service_timeout)

	# Defines a service for arming with topic and message
	global arming_client
	arming_client = rospy.ServiceProxy(uav_id + "/mavros/cmd/arming", CommandBool)   

	# Waits for the services to be available for each UAV
	rospy.wait_for_service(uav_id + "/mavros/set_mode", service_timeout) 

	# Defines a service for changing mode with topic and message
	global set_mode_client
	set_mode_client = rospy.ServiceProxy(uav_id + "/mavros/set_mode", SetMode) 

	# Waits for the service to be available
	rospy.wait_for_service(uav_id + '/mavros/set_stream_rate', service_timeout)

	# Defines a service for changing stream rate for setpoint
	global set_stream_rate
	set_stream_rate = rospy.ServiceProxy(uav_id + "/mavros/set_stream_rate", StreamRate)

	rospy.loginfo('Services available for uav number'+ uav_id_number + '!')

# Sets up all the needed publishers and subscribers
def setup_pub_sub(uav_id):

	# UAV DATA SUBSCRIBERS #

	# Creates the subscriber to the state of each aircraft
	state_sub = rospy.Subscriber(uav_id + "/mavros/state", State, callback = state_cb)

	# Creates the subscriber for aircraft altitude
	altitude_sub = rospy.Subscriber(uav_id + '/mavros/altitude', Altitude, altitude_cb)

	# Creates the subscriber for extended state
	extended_state_sub = rospy.Subscriber(uav_id + '/mavros/extended_state', ExtendedState, extended_state_cb)

	# Creates the subscriber for target position points
	setpoint_raw_sub = rospy.Subscriber(uav_id + '/mavros/setpoint_raw/local', PositionTarget, setpoint_raw_cb)

	# Creates the subscriber for Odometry
	local_position_odom_sub = rospy.Subscriber(uav_id + '/mavros/local_position/odom', Odometry, local_position_odom_cb)

	# Creates the subscriber for local position
	local_position_sub = rospy.Subscriber(uav_id + '/mavros/local_position/pose', PoseStamped, local_position_cb)

	# Creates the subscriber for Imu ???
	#imu_sub0 = rospy.Subscriber("/mavros/imu/data", Imu, imu_cb0)

	# Creates the subscriber for data on Heads Up Display
	VFR_HUD_sub = rospy.Subscriber(uav_id + '/mavros/vfr_hud', VFR_HUD, vfr_hud_cb)

	# Creates the subscriber for Attitude
	att_sub = rospy.Subscriber(uav_id + "/mavros/setpoint_raw/target_attitude", AttitudeTarget, attitude_cb)

	# TARGET #
	# Creates the subscriber for the target position
	target_sub = rospy.Subscriber("target_position", TargetTelemetry, receive_target_cb) # Might change in the future, if the objective is to get the images from each UAV


	# VIRTUAL PARTICLES #
	#virtual_particle_sub = rospy.Subscriber(str(int(uav_id_number) + 1) + "/particle_position", ParticleTelemetry, receive_particle_cb)
	virtual_particle_sub = rospy.Subscriber('uav' + str(int(uav_id_number) + 1) + "/particle_position", ParticleTelemetry, receive_particle_cb)

	# UAV PUBLISHERS #

	# Creates the publisher to publish setpoint
	local_pos_pub = rospy.Publisher(uav_id + "/mavros/setpoint_position/local", PoseStamped, queue_size=20)

	# Creates the publisher to publish attitude commands
	global attitude_pub
	attitude_pub = rospy.Publisher(uav_id + "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=20)

	# Creates the publisher for simulation data
	global MPF_data_pub
	MPF_data_pub = rospy.Publisher('MPF_data', MPF_data, queue_size=20)

	rospy.loginfo('Publishers and subscribers created for uav number '+ uav_id_number + '!')

# Function to publish attitudes

def send_att():

	global e, v_cmd, roll, thrust, pitch, heading_rate_cmd, delta_t
	att = AttitudeTarget()
	att.body_rate = Vector3()
	att.header = Header()
	att.header.frame_id = "base_footprint"

	# Function to calculate the roll given the current UAV data, position of the target and the virtual particle to follow.
	var = roll_control.roll_calculation(local_position, vfr_hud, target, particle, delta_t) 

	roll = var[0]
	e = var[1]

	# Needed for thrust control calculation
	v_cmd = var[2]

	# To plot
	heading_rate_cmd = var[3]

	# Calculates pitch and thrust inputs
	pitch_and_thrust = calculate_pitch_thrust(v_cmd)
	thrust = pitch_and_thrust[0]
	pitch = pitch_and_thrust[1]

	e_x = e[0]
	e_y = e[1]
	bank = (180*roll)/math.pi


	att.orientation = Quaternion(*quaternion_from_euler(roll, -pitch, 0))  # Coordinates in rads
	att.thrust = thrust
	att.type_mask = 7  # Ignore body rate
	att.header.stamp = rospy.Time.now()

	attitude_pub.publish(att)

# Function to calculate the thrust and pitch
def calculate_pitch_thrust(v_cmd):

	global delta_t
	# Calculates the thrust
	thrust = thrust_control.thrust_input(vfr_hud, delta_t, v_cmd)
	# Calculates the pitch
	pitch = pitch_control.compute_pitch(altitude, delta_t)

	return thrust, pitch

# Callback functions for each subscribed topic

def state_cb(data):
	global current_state
	current_state = data
	mode = data.mode

def altitude_cb(data):
	global altitude
	altitude = data

def attitude_cb(data):
	global att
	att = data

def extended_state_cb(data):
	pass

def setpoint_raw_cb(data):
	pass

def local_position_cb(data):
	global local_position
	local_position = data

def local_position_odom_cb(data):
	global local_position_odom
	local_position_odom = data

def vfr_hud_cb(data):
	global vfr_hud
	vfr_hud = data

def receive_target_cb(data):
	global target
	target = data

def receive_particle_cb(data):
	global particle
	particle = data

# def imu_cb(data):
#    imu = data
#    current_yaw = self.extract_yaw_from_quaternion(self.imu.orientation)

# MAIN FUNCTION #########

if __name__ == "__main__":

	rospy.init_node("takeoff_node_py", anonymous=True)   

	global uav_id, uav_id_number

	uav_id = rospy.get_param("~uav_id")
	uav_id_number = uav_id[-1]

	setup_services(uav_id) # Service setup

	setup_pub_sub(uav_id) # Publishers and subscribers setup

	# Variables initialization and definitions
	current_yaw = None
	current_target_pose = PositionTarget()
	current_pose = Point()
	current_state = State()

	home_pose = Point(0.0, 0.0, 0.0)
	altitude = Altitude()
	global_position = HomePosition()
	local_position = PoseStamped()  # FCU local position
	local_position_odom = Odometry()
	extended_state = ExtendedState()
	mode = ''
	att = AttitudeTarget()
	vfr_hud = VFR_HUD()
	hil_controls = ActuatorControl()
	#imu = Imu()

	# Target initialization
	target = TargetTelemetry()

	global e, roll, v_cmd

	# Particle initialization
	particle = ParticleTelemetry()

	# Thrust and pitch initialization
	thrust = 0.0
	pitch = 0.0

	# Delta t initialization
	global delta_t, previous_time
	delta_t = 0
	previous_time = rospy.get_time()

	# Initializes and saves the simulation data
	e = 0
	bank = 0

	MPF_Data = MPF_data()

	# Setpoint publishing MUST be faster than 2Hz
	rate = rospy.Rate(20)

	# Wait for Flight Controller connection
	rospy.loginfo('Waiting to connect...')
	while not rospy.is_shutdown() and not current_state.connected:
		rate.sleep()
	rospy.loginfo('Connected!')

	# Waits 20 seconds for take off, and then it can initiate the offboard mode
	time.sleep(5)

	# Initiates takeoff and arming sequence
	set_mode_client(0, 'AUTO.TAKEOFF') # Sets TAKEOFF mode for UAV0
	rospy.loginfo('TAKEOFF mode enabled')

	arming_client(True) # Arms the drone
	rospy.loginfo('Vehicle ' + uav_id_number + ' armed. Taking off...')

	# Defines message to be sent
	pose = PoseStamped()

	pose.pose.position.x = 500
	pose.pose.position.y = 500
	pose.pose.position.z = 50

	attitude_des = AttitudeTarget()
	attitude_des.body_rate = Vector3()
	attitude_des.header = Header()
	attitude_des.header.frame_id = "base_footprint"
	attitude_des.thrust = 0.3
	attitude_des.type_mask = 128  # 0b00000000 # Ignores roll and pitch rate
	attitude_des.body_rate.x = 0.1 
	attitude_des.body_rate.y = 0.0
	attitude_des.body_rate.z = 0.0

	global start_time
	start_time = rospy.get_time()
	
	if os.path.isfile("/home/mgfelix/catkin_ws/src/plot/simulation_data/data" + uav_id_number + ".csv"):
		os.remove("/home/mgfelix/catkin_ws/src/plot/simulation_data/data" + uav_id_number + ".csv")

	# Changes the stream rate
	set_stream_rate(3, 10, 1)

	# Loops to stay in offboard mode and start offboard control
	while not rospy.is_shutdown():
		attitude_des.header.stamp = rospy.Time.now()

		attitude_pub.publish(attitude_des) # Publish attitude to be set on UAV

		if current_state.mode != 'OFFBOARD':
			set_mode_client(0, 'OFFBOARD')  # Sets OFFBOARD mode

		start_offboard_control()
            
		rate.sleep()

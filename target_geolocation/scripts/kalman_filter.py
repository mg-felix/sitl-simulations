#! /usr/bin/env python3
import math
from MARS_msgs.msg import TargetTelemetry
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.srv import StreamRateRequest, StreamRate
import numpy as np
from numpy.linalg import inv
import rospy
from mavros_msgs.msg import State
import csv
import os

def filter(pos_x,pos_y, dt1, filter_x , filter_P):
    # dt = 0.1
    # Initialize State
    # dt=0.1
    dt=0.05
    filter.A = np.array([[1, dt, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, dt],
                          [0, 0, 0, 1]])

    filter.H = np.array([[1, 0 , 0, 0],
                         [0, 0, 1, 0]])

    filter.HT = np.array([[1, 0],
                         [0, 0],
                         [0, 1],
                         [0, 0]])

    # filter.R = np.array([[30, 0],
    #                      [0, 30]])

    filter.R = np.array([[60, 0],
                         [0, 60]])

    # filter.Q = np.array([[0.001, 0, 0, 0, 0, 0],
    #                      [0, 0.0001, 0, 0, 0, 0],
    #                      [0, 0, 0.0001, 0, 0, 0],
    #                      [0, 0, 0, 0.001, 0, 0],
    #                      [0, 0, 0, 0, 0.0001, 0],
    #                      [0, 0, 0, 0, 0, 0.0001]])

    filter.Q = np.array([[0.0001, 0, 0, 0],
                         [0, 0.00001, 0, 0],
                         [0, 0, 0.0001, 0],
                         [0, 0, 0, 0.0001]])

    # Predict State Forward
    x_p = np.dot(filter.A,filter_x)
    # Predict Covariance Forward
    P_p = filter.A.dot(filter_P).dot(filter.A.T) + filter.Q
    # Compute Kalman Gain
    S = np.dot((np.dot(filter.H,P_p)),filter.HT) + filter.R
    K = np.dot((np.dot(P_p,filter.HT)),(inv(S)))

    if len(pos_x):
        rospy.logwarn_throttle(5,'Kalman filter estimate!')
        z = np.array([pos_x[0],pos_y[0]])
        # Estimate State
        residual = z - np.dot(filter.H, x_p)
        filter.x = x_p + np.dot(K,residual)
        # Estimate Covariance
        filter_P = P_p - np.dot((np.dot(K,filter.H)),P_p)
    else:
        rospy.logfatal_throttle(5,'Kalman filter prevision!')
        filter.x = x_p
        # Estimate Covariance
        filter_P = P_p

    return filter.x, filter_P

class kalman_filter(object):

    def __init__(self):
        self.x_pred = np.array([0, 0, 0, 0])
        self.P_pred = 0.01 * np.eye(len(self.x_pred))
        self.dt = 0.0
        self.psi_anterior = 0.0
        self.x_true_anterior = 0
        self.y_true_anterior = 0
        self.target_true_position = Point()
        self.rate = rospy.Rate(20)
        self.service_timeout = 30
        self.setup_pubsub()
        self.setup_services()
        self.state = State()
        self.target_position_geolocation = TargetTelemetry()
        self.local_position = PoseStamped()

    def setup_pubsub(self):
        rospy.loginfo("-------Setting pub - sub-----")
        self.targetposition_geo_sub = rospy.Subscriber(uav_id + "/target_position_geolocation", TargetTelemetry, self.target_position_geolocation_cb)
        self.animated_box_sub = rospy.Subscriber("/animated_box", Point, self.target_true_position_cb)
        self.state_sub = rospy.Subscriber(uav_id + "/mavros/state", State, self.state_cb)
        self.local_position_sub = rospy.Subscriber(uav_id + "/mavros/local_position/pose", PoseStamped, self.local_position_cb)

    # ros services
    def setup_services(self):
        rospy.loginfo('----Waiting for services to connect----')
        try:
            rospy.wait_for_service(uav_id + '/mavros/set_stream_rate', self.service_timeout)
            rospy.loginfo('Services are connected and ready')
        except rospy.ROSException as e:
            rospy.logerr('Failed to initialize service')

        # Set services
        self.set_stream_rate_srv = rospy.ServiceProxy(uav_id + '/mavros/set_stream_rate', StreamRate)

        '''set mavros stream rate to get mavros messages faster.
        mavros publishes state/setpoint messages at 1 hz by default
        '''
        rospy.loginfo('setup services finished')

    def set_mavros_stream_rate(self):
        stream_rate = StreamRateRequest()
        stream_rate.request.stream_id = 3
        stream_rate.request.message_rate = 10
        stream_rate.request.on_off = 1
        try:
            self.set_stream_rate_srv(stream_rate)
        except rospy.ServiceException as exp:
            rospy.logerr('Stream rate service failed')
        rospy.loginfo('setup mavros stream rate finished')

    def start(self):
        # wait to get heartbeat from fcu
        while not self.state.connected:
            self.rate.sleep()

        pub = rospy.Publisher("/target_position", TargetTelemetry, queue_size=10)
        target_3d_position_msg = TargetTelemetry()
        curr_time = rospy.get_time()
        rospy.loginfo('--Got heartbeat from FCU----')
        start_time = rospy.get_time()
        while not rospy.is_shutdown():

            self.dt = rospy.get_time() - curr_time
            curr_time = rospy.get_time()

            if self.target_position_geolocation.x_pos == 0 and self.target_position_geolocation.y_pos == 0:
                target_data = filter([],[], self.dt, self.x_pred,
                                                          self.P_pred)
            else:
                target_data = filter([self.target_position_geolocation.x_pos], [self.target_position_geolocation.y_pos], self.dt,
                                     self.x_pred,
                                     self.P_pred)

            self.x_pred = target_data[0]
            self.P_pred = target_data[1]

            estimation_data = target_data[0]

            # estimacao psi e omega
            psi_estimation = math.atan(estimation_data[3] / estimation_data[1])
            # psi_estimation wraptopi
            while psi_estimation < -math.pi:
                psi_estimation = psi_estimation + 2 * math.pi

            while psi_estimation > math.pi:
                psi_estimation = psi_estimation - 2 * math.pi

            dif_psi = psi_estimation - self.psi_anterior

            while dif_psi < -math.pi:
                dif_psi = dif_psi + 2 * math.pi

            while dif_psi > math.pi:
                dif_psi = dif_psi - 2 * math.pi

            if self.dt == 0:
                self.dt = 0.0001
            omega_estimation = dif_psi / self.dt

            omega = np.tanh(omega_estimation)

            self.psi_anterior = psi_estimation

            # calculo do tempo para o plot
            time = rospy.get_time()
            time_plot = time - start_time
            x_true = self.target_true_position.x
            y_true = self.target_true_position.y
            vx_true = (x_true - self.x_true_anterior) / self.dt
            vy_true = (y_true - self.y_true_anterior) / self.dt
            self.x_true_anterior = x_true
            self.y_true_anterior = y_true
            v_true = math.sqrt(vx_true ** 2 + vy_true ** 2)
            # calculo dos erros de posicao
            # erro_posicao_kalman = math.sqrt((x_true - estimation_data[0]) ** 2 + (y_true - estimation_data[3]) ** 2)
            erro_posicao_kalman = math.sqrt((0 - estimation_data[0]) ** 2 + (0 - estimation_data[2]) ** 2)


            acc = 0
            vel = math.sqrt(estimation_data[1] ** 2 + estimation_data[3] ** 2)
            psi = psi_estimation

            target_3d_position_msg.x_pos = estimation_data[0]
            target_3d_position_msg.y_pos = estimation_data[2]
            target_3d_position_msg.vx = estimation_data[1]
            target_3d_position_msg.vy = estimation_data[3]
            target_3d_position_msg.omega = omega
            target_3d_position_msg.accel = 0
            target_3d_position_msg.vel = vel
            target_3d_position_msg.psi = psi

            pub.publish(target_3d_position_msg)
            rospy.loginfo_throttle(5,target_3d_position_msg.x_pos)
            rospy.loginfo_throttle(5,target_3d_position_msg.y_pos)
            rospy.loginfo_throttle(5,target_3d_position_msg.vx)
            rospy.loginfo_throttle(5,target_3d_position_msg.vy)
            

            d = open("/home/mgfelix/catkin_ws/src/plot/simulation_data/target.csv", "a")

            with d:
                writer = csv.writer(d)
                writer.writerow([time_plot,
                                 self.local_position.pose.position.x,
                                 self.local_position.pose.position.y,
                                 target_3d_position_msg.x_pos,
                                 target_3d_position_msg.y_pos,
                                 target_3d_position_msg.vx,
                                 target_3d_position_msg.vy,
                                 target_3d_position_msg.omega,
                                 target_3d_position_msg.accel,
                                 target_3d_position_msg.vel,
                                 target_3d_position_msg.psi,
                                 x_true,
                                 y_true,
                                 vx_true,
                                 vy_true,
                                 v_true,
                                 erro_posicao_kalman,
                                 ])

            self.rate.sleep()

    def target_position_geolocation_cb(self,data):
        self.target_position_geolocation = data

    def target_true_position_cb(self, data):
            self.target_true_position = data

    def state_cb(self, data):
        self.state = data
        self.mode = data.mode

    def local_position_cb(self, data):
        self.local_position = data

if __name__ == '__main__':
    rospy.init_node('kalman_filter', anonymous=True)

    global uav_id_number, uav_id
    uav_id = rospy.get_param("~uav_id")
    uav_id_number = uav_id[-1]

    if os.path.isfile("/home/mgfelix/catkin_ws/src/plot/simulation_data/target.csv"):
        os.remove("/home/mgfelix/catkin_ws/src/plot/simulation_data/target.csv")
   
    rospy.sleep(0)

    try:
        inicializacao = kalman_filter()
        inicializacao.start()

        rospy.spin()

    except rospy.ROSInterruptException as exception:
        pass

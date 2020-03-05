#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import tf
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, F = None, H = None, B = None, P = None, Q = None, R = None):
        self.n_state = F.shape[0] # dimension of state
        self.n_observ = H.shape[0] # dimension of observation
        self.n_control = B.shape[0] # dimension of control

        self.F = F # state transition model n*n
        self.H = H # observation model k*n
        self.B = B # control-input model n*l
        self.Q = np.eye(self.n_state) if Q is None else Q # covariance of process noise
        self.R = np.eye(self.n_state) if R is None else R # covariance of observation noise
        self.P = np.eye(self.n_state) if P is None else P # posteriori estimate covariance matrix 
        self.x = np.zeros((self.n_state, 1)) # state x_t/t
        self.x_predict = np.zeros((self.n_state, 1)) # predict state x_t+1/t

        self.z = np.zeros((self.n_observ, 1)) # sensor reading
        self.u = np.zeros((self.n_control, 1)) # control

    def predict(self):
        self.x_predict = self.F * self.x + self.B * self.u
        self.P = self.F * self.P * self.F.T +self.Q

    def update(self):
        r = self.z - self.H * self.x # difference between expected and "true"
        S = self.H * self.P * self.H.T + self.R # covariance of sensor reading
        K = self.P * self.H.T * S.T # Kalman gain
        self.x = self.x_predict + K * r
        I = np.eye(self.n_state)
        self.P = (I - np.dot(K, self.H)) * self.P

class Server:
    def __init__(self):
        rospy.init_node('pose_vo')
        self.imusub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_callback)
        self.posepub = rospy.Publisher('pose_kf_imu', Odometry, queue_size = 0)
        self.pose_kf = Odometry()

        self.n_state = 12 # dimension of state
        self.n_observ = 6 # dimension of observation
        self.n_control = 1 # dimension of control

        B = np.eye(self.)
        F = np.array([1])
        H = np.array([1])
        Q = np.array([1])
        P = np.array([1000])
        R = np.array([1])

        self.kf = KalmanFilter(F, H, B, P, Q, R)

        self.t = []
        self.t.append(0)
        self.init_time = rospy.Time.now()
        self.state = []

    def imu_callback(self, msg):

        self.kf.predict()
        self.kf.update()

        self.t.append(rospy.Time.now() - self.init_time)
        self.state.append(self.kf.x)

        self.pose_kf.header.frame_id = "odom_kf"
        self.pose_kf.header.seq = 0
        self.pose_kf.header.stamp = rospy.Time.now()

        self.pose_kf.pose.pose.position.x = self.state[0]
        self.pose_kf.pose.pose.position.y = self.state[1]
        self.pose_kf.pose.pose.position.z = self.state[2]
        self.pose_kf.pose.pose.orientation.x = self.state[3]
        self.pose_kf.pose.pose.orientation.y = self.state[4]
        self.pose_kf.pose.pose.orientation.z = self.state[5]
        self.pose_kf.twist.twist.linear.x = self.state[6]
        self.pose_kf.twist.twist.linear.y = self.state[7]
        self.pose_kf.twist.twist.linear.z = self.state[8]
        self.pose_kf.twist.twist.angular.x = self.state[9]
        self.pose_kf.twist.twist.angular.y = self.state[10]
        self.pose_kf.twist.twist.angular.z = self.state[11]

        self.pose_kf.pose.covariance = self.P[0:5, 0:5]
        self.pose_kf.twist.covariance = self.P[6:11, 6:11]

        self.posepub.publish(self.pose_kf)

    def plot(self):
        plt.plot(self.t, self.x, label = 'x from kalman filter')
        plt.plot(self.t, [0]*len(t), 'r', linestyle='solid', label = 'ground truth')
        plt.title("Kalman Filter with IMU")
        plt.xlabel('time(s)')
        plt.ylabel('distance(m)')
        plt.legend(loc='upper left')
        #plt.text(3, 1, "error: "+str(1.1 - kf.x))

        plt.show()
        

if __name__ == "__main__":
    server = Server()    


#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
import numpy as np
import tf.transformations
import tf
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from pynput import keyboard
import math

RATE = 30
IMU_SIGMA = 1e-4
VO_SIGMA = 0.01
END_TIME = 22

class VO:
    def __init__(self):
        rospy.init_node('VO_stag')
        tag_sub = rospy.Subscriber("bluerov_controller/ar_tag_detector", AlvarMarkers, self.tag_callback)
        path_sub = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.path_callback)
        self.posepub = rospy.Publisher('pose_vo', Odometry, queue_size = 0)
        imusub = rospy.Subscriber('imu/data', Imu, self.imu_callback)
        imu_raw_sub = rospy.Subscriber('mavros/imu/data_raw', Imu, self.imu_raw_callback)
        self.imupub = rospy.Publisher('imu/data_raw', Imu, queue_size=0)
        self.imu_modified_pub = rospy.Publisher('imu/modified_data', Imu, queue_size=0)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.tag_exist = False
        self.markers = []
        self.rate = rospy.Rate(RATE)

        self.init_trans = []
        self.init_rot = []
        self.combine_init_trans = []
        self.pose_is_init = False

        self.init_time = rospy.Time.now().secs
        self.vo_init_time = 0
        self.t_vo = []
        self.t = []
        self.t_imu = []

        self.x = []
        self.y = []
        self.z = []
        self.x_vo = []
        self.y_vo = []
        self.z_vo = []
        self.yaw = []
        self.pitch = []
        self.roll = []
        self.yaw_vo = []
        self.pitch_vo = []
        self.roll_vo = []
        self.yaw_imu = []
        self.pitch_imu = []
        self.roll_imu = []

        self.init_imu = False
        self.last_imu_time = 0

        self.get_init_time = False
        self.start_vo = False

        self.finish = False

    def path_callback(self, msg):
        if self.start_vo:
            pose = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            orientation = [ori.x, ori.y, ori.z, ori.w]
            euler = tf.transformations.euler_from_quaternion(orientation)
            self.t.append(rospy.Time.now().to_sec() - self.vo_init_time)
            self.x.append(pose.x)
            self.y.append(pose.y)
            self.z.append(pose.z)
            self.roll.append(euler[0])
            self.pitch.append(euler[1])
            self.yaw.append(euler[2])

    def imu_raw_callback(self, msg):
        if self.start_vo:
            imu_msg = Imu()
            imu_msg.header = msg.header
            imu_msg.angular_velocity = msg.angular_velocity
            imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            imu_msg.linear_acceleration = msg.linear_acceleration
            imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            imu_msg.header.frame_id = "imu"
            for i in range(0, 9, 4):
                imu_msg.orientation_covariance[i] = IMU_SIGMA**2
            self.imupub.publish(imu_msg)

    def imu_callback(self, msg):
        if self.start_vo:
            ori = msg.orientation
            orientation = [ori.x, ori.y, ori.z, ori.w]
            euler = tf.transformations.euler_from_quaternion(orientation)
            self.t_imu.append(rospy.Time.now().to_sec() - self.vo_init_time)
            self.roll_imu.append(euler[0]+1.62)
            self.pitch_imu.append(euler[1])
            self.yaw_imu.append(euler[2])
            #print(msg.angular_velocity.x, euler[0])

            ori = tf.transformations.quaternion_from_euler(euler[0]+1.62, euler[1], euler[2])
            imu_msg = Imu()
            imu_msg.header = msg.header
            imu_msg.angular_velocity = msg.angular_velocity
            imu_msg.angular_velocity_covariance = msg.angular_velocity_covariance
            imu_msg.linear_acceleration = msg.linear_acceleration
            imu_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance
            imu_msg.orientation_covariance = msg.orientation_covariance
            imu_msg.orientation.x = ori[0]
            imu_msg.orientation.y = ori[1]
            imu_msg.orientation.z = ori[2]
            imu_msg.orientation.w = ori[3]
            self.imu_modified_pub.publish(imu_msg)

    def tag_callback(self, msg):
        if len(msg.markers) != 0:
            print("detect marker")
            self.tag_exist = True
            marker = msg.markers[0]

            if not self.get_init_time:
                self.init_time = marker.header.stamp.secs
                self.get_init_time = True
                print("get init time", self.init_time)

            duration = marker.header.stamp.secs - self.init_time
            print("duration:"+str(duration)+"s")
            # the tag is shaking in the first few seconds
            if not self.start_vo:
                self.start_vo = True
                self.vo_init_time = rospy.Time.now().to_sec()
            if duration > END_TIME:
                self.finish = True

        else: self.tag_exist = False

    def relative_transformation(self, trans, rot):
        for i in range(3):
            trans[i] = trans[i] - self.init_trans[i]

        # dq = q_final * inverse(q_init)
        rot = tf.transformations.quaternion_multiply(rot, tf.transformations.quaternion_inverse(self.init_rot))

        return trans, rot

    def inverse_transformation(self, trans, rot):
        R = self.quat2R(rot)
        R = np.linalg.inv(R)
        rot = self.R2quat(R)

        trans = np.array([[trans[0]], [trans[1]], [trans[2]]])
        trans = -R.dot(trans)
        trans = [trans[0, 0], trans[1, 0], trans[2, 0]]
        #print(trans)

        return trans, rot

    def quat2R(self, quat):
        q0 = quat[3]
        q1 = quat[0]
        q2 = quat[1]
        q3 = quat[2]
        R = np.matrix([[1-2*q2**2-2*q3**2, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2],
                       [2*q1*q2+2*q0*q3, 1-2*q1**2-2*q3**2, 2*q2*q3-2*q0*q1],
                       [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*q1**2-2*q2**2]])
        return R

    def R2quat(self, R):
        q0 = math.sqrt(1+R[0,0]+R[1,1]+R[2,2])/2
        q1 = (R[2,1]-R[1,2])/(4*q0)
        q2 = (R[0,2]-R[2,0])/(4*q0)
        q3 = (R[1,0]-R[0,1])/(4*q0)

        rot = [q1, q2, q3, q0]
        return rot

if __name__ == "__main__":
    vo = VO()
    while not rospy.is_shutdown():
        try:
            if not vo.pose_is_init:
                (vo.init_trans, vo.init_rot) = vo.listener.lookupTransform("/ar_marker_1", "imu", rospy.Time(0))
                vo.pose_is_init = True
                print("pose is init")
                print(vo.init_trans)

            # tag fix
            (trans, rot) = vo.listener.lookupTransform( "/ar_marker_1", "/imu", rospy.Time(0))

            # get the relative transformation to the init pose
            (trans, rot) = vo.relative_transformation(trans, rot)

            # publish Visual Odometry message to ekf
            vo_msg = Odometry()

            vo_msg.header.frame_id = "/ar_marker_1"
            vo_msg.header.seq = 0
            vo_msg.header.stamp = rospy.Time.now()

            # vo_msg.pose.pose.position.x = -trans[0]
            # vo_msg.pose.pose.position.z = -trans[1]
            # vo_msg.pose.pose.position.y = trans[2]
            relative_trans = [trans[0] - vo.init_trans[0],trans[1] - vo.init_trans[1],trans[2] - vo.init_trans[2]]
            vo_msg.pose.pose.position.x = trans[0]
            vo_msg.pose.pose.position.y = trans[1]
            vo_msg.pose.pose.position.z = trans[2]
            vo_msg.pose.pose.orientation.x = rot[0]
            vo_msg.pose.pose.orientation.y = rot[1]
            vo_msg.pose.pose.orientation.z = rot[2]
            vo_msg.pose.pose.orientation.w = rot[3]

            for i in range(0, 36, 7):
                vo_msg.pose.covariance[i] = VO_SIGMA**2

            vo.posepub.publish(vo_msg)

            if vo.start_vo:
                # vo.x_vo.append(-trans[0])
                # vo.z_vo.append(-trans[1])
                # vo.y_vo.append(trans[2])
                vo.x_vo.append(trans[0])
                vo.y_vo.append(trans[1])
                vo.z_vo.append(trans[2])

                euler = tf.transformations.euler_from_quaternion(rot)
                vo.roll_vo.append(rot[0])
                vo.pitch_vo.append(rot[1])
                vo.yaw_vo.append(rot[2])

                vo.t_vo.append(rospy.Time.now().to_sec() - vo.vo_init_time)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #print("no transformation")
            pass      

        vo.rate.sleep()

        if vo.finish:
            # plot 3D path
            fig = plt.figure()
            ax = Axes3D(fig)
            ax.plot(vo.x, vo.y, vo.z, label="vio")
            ax.plot(vo.x_vo, vo.y_vo, vo.z_vo, label="vo")
            plt.legend()

            # plot orientation
            plt.figure()
            plt.subplot(311)
            plt.plot(vo.t, vo.roll, label="vio")
            plt.plot(vo.t_vo, vo.roll_vo, label="vo")
            plt.plot(vo.t_imu, vo.roll_imu, label="imu")
            plt.legend()
            plt.subplot(312)
            plt.plot(vo.t, vo.pitch, label="vio")
            plt.plot(vo.t_vo, vo.pitch_vo, label="vo")
            plt.plot(vo.t_imu, vo.pitch_imu, label="imu")
            plt.legend()
            plt.subplot(313)
            plt.plot(vo.t, vo.yaw, label="vio")
            plt.plot(vo.t_vo, vo.yaw_vo, label="vo")
            plt.plot(vo.t_imu, vo.yaw_imu, label="imu")
            plt.legend()

            # plot position
            plt.figure()
            plt.subplot(311)
            plt.plot(vo.t, vo.x, label="vio")
            plt.plot(vo.t_vo, vo.x_vo, label="vo")
            plt.legend()
            plt.subplot(312)
            plt.plot(vo.t, vo.y, label="vio")
            plt.plot(vo.t_vo, vo.y_vo, label="vo")
            plt.legend()
            plt.subplot(313)
            plt.plot(vo.t, vo.z, label="vio")
            plt.plot(vo.t_vo, vo.z_vo, label="vo")
            plt.legend()
            plt.show()
            break
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
IMU_SIGMA = 0.01
VO_SIGMA = 0.01

class STag:
    def __init__(self, tagid, x, y, z, roll, pitch, yaw):
        self.id = tagid
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class VO:
    def __init__(self):
        rospy.init_node('VO_stag')
        tag_sub = rospy.Subscriber("bluerov_controller/ar_tag_detector", AlvarMarkers, self.tag_callback)
        path_sub = rospy.Subscriber("robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.path_callback)
        self.posepub = rospy.Publisher('pose_vo', Odometry, queue_size = 0)
        #imusub = rospy.Subscriber('imu/data', Imu, self.imu_callback)
        #self.imupub = rospy.Publisher('imu_data', Imu, queue_size=0)

        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        self.tag_exist = False
        self.markers = []
        self.rate = rospy.Rate(RATE)

        self.init_trans = []
        self.init_rot = []
        self.pose_is_init = False

        self.init_time = rospy.Time.now().to_sec()
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

        #self.init_time = 0
        self.get_init_time = False
        self.start_vo = False

        self.finish = False
        keyboard_listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        keyboard_listener.start() 

    def is_space_key(self, key):
        return key == keyboard.Key.space

    # responds to keyboard events
    def on_press(self, key):
        if self.is_space_key(key):
            pass

    def on_release(self, key):
        if self.is_space_key(key):
            self.finish = True

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

    def imu_callback(self, msg):
        if self.start_vo:
            ori = msg.orientation
            orientation = [ori.x, ori.y, ori.z, ori.w]
            euler = tf.transformations.euler_from_quaternion(orientation)
            self.t_imu.append(rospy.Time.now().to_sec() - self.vo_init_time)
            self.roll_imu.append(euler[0])
            self.pitch_imu.append(euler[1])
            self.yaw_imu.append(euler[2])

        # if not self.init_imu:
        #     self.last_imu_time = msg.header.stamp.to_sec()
        #     self.init_imu = True
        #     return

        # imu_msg = Imu()
        # imu_msg.header = msg.header
        # imu_msg.header.frame_id = "imu"
        # dt = (msg.header.stamp.nsecs - self.last_imu_time)*1e-9
        # self.last_imu_time = msg.header.stamp.nsecs

        # ang_v = msg.angular_velocity
        # q_rot = tf.transformations.quaternion_from_euler(ang_v.x, ang_v.y, ang_v.z)
        # ori = msg.orientation
        # q_ori = [ori.x, ori.y, ori.z, ori.w]
        # q_d = tf.transformations.quaternion_multiply(q_ori, q_rot)

        # imu_msg.orientation.x = q_d[0]*dt*0.5
        # imu_msg.orientation.y = q_d[1]*dt*0.5
        # imu_msg.orientation.z = q_d[2]*dt*0.5
        # imu_msg.orientation.w = q_d[3]*dt*0.5

        # for i in range(0, 9, 4):
        #     imu_msg.orientation_covariance[i] = IMU_SIGMA**2

        #self.imupub.publish(imu_msg)


    def tag_callback(self, msg):
        if len(msg.markers) != 0:
            print("detect marker")
            self.tag_exist = True
            marker = msg.markers[0]
            # tagid = marker.id
            # pose = marker.pose.pose.position
            # x = pose.x
            # y = pose.y
            # z = pose.z
            # orientation = marker.pose.pose.orientation
            # orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
            # (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            #print(self.markers)
        
            #vo_msg = Odometry()
            #self.posepub.publish(vo_msg)

            if not self.get_init_time:
                self.init_time = marker.header.stamp.secs
                self.get_init_time = True
                print("get init time", self.init_time)

            duration = marker.header.stamp.to_sec() - self.init_time
            print("duration:"+str(duration)+"s")
            # the tag is shaking in the first few seconds
            if duration > 8 and not self.start_vo:
                self.start_vo = True
                self.vo_init_time = rospy.Time.now().to_sec()
            if duration > 22:
                self.finish = True

        else: self.tag_exist = False

    def relative_transformation(self, trans, rot):
        T1 = self.transformation_matrix(trans, self.quat2R(rot))
        T0 = self.transformation_matrix(self.init_trans, self.quat2R(self.init_rot))

        T = np.matmul(T0, T1)
        R = T[0:3, 0:3]
        
        trans = [T[0,3], T[1,3], T[2,3]]
        rot = self.R2quat(R)

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


    def transformation_matrix(self, trans, R):
        T = np.identity(4)
        T[0:3, 0:3] = R
        T[0,3] = trans[0]
        T[1,3] = trans[1]
        T[2,3] = trans[2]

        return T

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
                (vo.init_trans, vo.init_rot) = vo.listener.lookupTransform("/imu", "/ar_marker_1", rospy.Time(0))
                vo.pose_is_init = True
                print("pose is init")
                #vo.broadcaster.sendTransform(vo.init_trans, vo.init_rot, rospy.Time(0),"/ar_marker_1", "/odom_vo")
            
            #vo.broadcaster.sendTransform(vo.init_trans, vo.init_rot, rospy.get_rostime(), "/ar_marker_1", "/odom_vo")

            # tag fix
            #(trans, rot) = vo.listener.lookupTransform("/cam_0_optical_frame", "/ar_marker_1", rospy.Time(0))
            (trans, rot) = vo.listener.lookupTransform( "/ar_marker_1", "/imu", rospy.Time(0))
            #print(trans, rot)

            #(trans, rot) = vo.relative_transformation(trans, rot)
            #(trans, rot) = vo.inverse_transformation(trans, rot)

            #print(trans, rot)

            vo_msg = Odometry()

            vo_msg.header.frame_id = "/ar_marker_1"
            vo_msg.header.seq = 0
            vo_msg.header.stamp = rospy.Time.now()

            # vo_msg.pose.pose.position.x = -trans[0]
            # vo_msg.pose.pose.position.z = -trans[1]
            # vo_msg.pose.pose.position.y = trans[2]
            vo_msg.pose.pose.position.x = trans[0]
            vo_msg.pose.pose.position.y = trans[1]
            vo_msg.pose.pose.position.z = trans[2]
            vo_msg.pose.pose.orientation.x = rot[0]
            vo_msg.pose.pose.orientation.y = rot[1]
            vo_msg.pose.pose.orientation.z = rot[2]
            vo_msg.pose.pose.orientation.w = rot[3]

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



            # vo_msg.twist.twist.linear.x = self.state[6]
            # vo_msg.twist.twist.linear.y = self.state[7]
            # vo_msg.twist.twist.linear.z = self.state[8]
            # vo_msg.twist.twist.angular.x = self.state[9]
            # vo_msg.twist.twist.angular.y = self.state[10]
            # vo_msg.twist.twist.angular.z = self.state[11]

            for i in range(0, 36, 7):
                vo_msg.pose.covariance[i] = VO_SIGMA**2
            #vo_msg.twist.covariance = 

            vo.posepub.publish(vo_msg)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #print("no transformation")
            pass      

        vo.rate.sleep()

        if vo.finish:
            fig = plt.figure()
            ax = Axes3D(fig)
            ax.plot(vo.x, vo.y, vo.z)
            ax.plot(vo.x_vo, vo.y_vo, vo.z_vo)
            #print(vo.x_vo, vo.y_vo, vo.z_vo)

            plt.figure()
            plt.subplot(311)
            plt.plot(vo.t, vo.roll)
            plt.plot(vo.t_vo, vo.roll_vo)
            #plt.plot(vo.t_imu, vo.roll_imu)
            plt.subplot(312)
            plt.plot(vo.t, vo.pitch)
            plt.plot(vo.t_vo, vo.pitch_vo)
            #plt.plot(vo.t_imu, vo.pitch_imu)
            plt.subplot(313)
            plt.plot(vo.t, vo.yaw)
            plt.plot(vo.t_vo, vo.yaw_vo)
            #plt.plot(vo.t_imu, vo.yaw_imu)

            plt.figure()
            plt.subplot(311)
            plt.plot(vo.t, vo.x)
            plt.plot(vo.t_vo, vo.x_vo)
            plt.subplot(312)
            plt.plot(vo.t, vo.y)
            plt.plot(vo.t_vo, vo.y_vo)
            plt.subplot(313)
            plt.plot(vo.t, vo.z)
            plt.plot(vo.t_vo, vo.z_vo)
            plt.show()
            break
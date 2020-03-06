## Combining STag and IMU with EKF package
This package is based on robot_pose_ekf, stag and imu_complementary_filter ros package. The stag package is to detect stag(a kind of fiducial marker) id and relative pose between marker and camera. The imu comlementary filter package is to get the orientation of the imu from raw imu data. A node is implemented to transform message from stag to visual odometry message by computing the transformation to the initial pose. Some comparing plots of path, translation and orienation of visual odometry and the final combined visual-inertial-odometry can also be seen after several seconds.

### Dependency
1. stag package  
https://github.com/dartmouthrobotics/stag
2. imu_complementary_filter package  
https://github.com/ccny-ros-pkg/imu_tools
3. ar_track_alvar package(for ar tag message. Copying their message folder in another package also works)
https://github.com/ros-perception/ar_track_alvar

### Set up
1. `catkin_make` and `source devel/setup.bash` in ros workspace
2. run launch file  
`roslaunch robot_pose_ekf robot_pose_ekf.launch`
3. play rosbag  
`rosbag play --clock _2020-03-02-15-30-29.bag`

# IMU_Cam_LiDAR_driver

For the data recording of Xsens IMU, Zed stereo camera and VLP16 LiDAR

Yulin Yang from yuyang@udel.edu

06/19/2017

Instruction for configuring xsens IMU, zed camera and velodyne LiDAR


#driver download: 

For velodyne: (recommended, with rviz for display)
github velodyne driver, ROS support for Velodyne 3D LIDARs 
- https://github.com/ros-drivers/velodyne.git

For zed camera:(recommended, with detailed instruciton for sensor configuration)
github rpng zed_cpu_ros, no need to install cuda
- https://github.com/rpng/zed_cpu_ros.git

For xsens driver
github ethz-asl (recommended)
- https://github.com/ethz-asl/ethzasl_xsens_driver.git

github xsens website (detailed instruction for sensor configuratioin)
- https://github.com/xsens/xsens_mti_ros_node.git

# build drivers:
1. setup the ros work space, like ~/catkin_ws_data with subfolders ~/catkin_ws_data/src, ~/catkin_ws_data/devel, ~/catkin_ws_data/build;
2. download all the source code into the folder of src;
3. 'catkin build'

Note: If missing any libs, just find out and install. 


# check sensors
1. setup the sensor
'rosrun xsens_driver mtdevice.py -m $sm -f $fs # publish sensor data'

where $fs can be 1,5,10,20,40,50,80,100,200 or 400Hz. This configures the MTi to output inertial data and magnetometer data at the set ODR. The maximum supported inertial update rate is 400Hz and for the magnetometer it is 100Hz. The $sm can be set to 1,2 or 3. This can be used to set the sensor to output sensor data or filter outputs.
% start each sensors
2. start xsens IMU
%newgrp dialout
'sudo chmod 777 /dev/ttyUSB0'
'source devel/setup.bash'
'roslaunch xsens_driver xsens_driver.launch'
3. check the imu data
'rostopic echo /imu/data'

% start the zed camera sensor
1. setup the sensor
all the parameters needed for setting up are in the launch file (the frequency, the calibration parameters)
2. `roslaunch zed_cpu_ros zed_cpu_ros.launch`
3. check the camera images
`/camera/left/image_raw`
`/camera/right/image_raw`

% start the velodyne LiDAR
1. `roslaunch velodyne_pointcloud VLP16_points.launch`
2. check the lidar data
`/velodyne_points`

# rosbag record data
TODO: write a launch file to launch all the sensors together 

1) record IMU, Camera and LiDAR data with selected rostopics
`rosbag record -O icl_xx /imu/data /camera/left/image_raw /camera/right/image_raw /velodyne_points`
2) record Camera and LiDAR with selected rostopics
`rosbag record -O cl_xx /camera/left/image_raw /camera/right/image_raw /velodyne_points`
3) record IMU and Camera data
`rosbag record -O ic_xx /imu/data /camera/left/image_raw /camera/right/image_raw`

4) record all the messages from IMU, Camera and LiDAR
`rosbag record -a`
4) record all the messages from camera and LiDAR
`rosbag record -a`
5) record all the messages from IMU and Camera
`rosbag record -a`







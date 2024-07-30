# Visual SLAM on Drone 

## Hardware Requirements 
* Intel NUC ([Link]()) with Ubuntu 20.04
* Intel Realsense D435i Depth camera ([Link]())
* USB 3.2 Gen 2 Type C to Type A cable 
* BEC for powering on NUC (Used Traco TEP 1000-2415 - [Link]())
* Flight Controller with Ardupilot (Used [Link]())
* Resisters 

## Software Requirements 
 ### Intel NUC
* ROS Noetic
* Ardupilot (Mavproxy) 
* Mavros
* Python3
* realsense2_camera -> ([Link](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy))
* imu_filter_madgwick
* rtabmap_ros
* robot_localization

### Flight controller

Param File inside the folder pixhawk


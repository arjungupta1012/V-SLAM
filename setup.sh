#!/bin/bash

# Update and upgrade system packages
sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get dist-upgrade -y

# Install essential packages
sudo apt-get install -y git wget cmake build-essential
sudo apt-get install -y libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# Clone and set up Librealsense
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false
sudo make uninstall && make clean && make -j6 && sudo make install
cd ../..

# Add ROS repository and install ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Setup ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install ROS dependencies
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

# Setup Catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional ROS packages
sudo apt-get install -y ros-noetic-imu-filter-madgwick
sudo apt-get install -y ros-noetic-rtabmap-ros
sudo apt-get install -y ros-noetic-robot-localization
sudo apt-get install -y ros-noetic-octomap-rviz-plugins

# Install MAVROS and geographic datasets
sudo apt-get install -y ros-noetic-mavros ros-noetic-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

# Install RealSense viewer
sudo apt-get install -y realsense-viewer


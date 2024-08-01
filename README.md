# Visual SLAM on Drone 

## Hardware Requirements 
* Intel NUC ([Link]()) with Ubuntu 20.04
* Intel Realsense D435i Depth camera ([Link]())
* USB 3.2 Gen 2 Type C to Type A cable 
* BEC for powering on NUC (Used Traco TEP 1000-2415 - [Link]())
* Flight Controller with Ardupilot (Used [Link]())
* Resisters 

## Prerequisites

1. **ROS (Robot Operating System)**: Ensure ROS is installed on your system. This guide assumes you are using ROS Noetic.
2. **MAVProxy**: A command-line ground control station for MAVLink-based drones.
3. **RealSense SDK**: Install the RealSense SDK for camera support.
4. **Python 3**: Required for running the Python scripts.
5. **Mavros**: Required for connecting Mavproxy to ROS

## Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/arjungupta1012/V-SLAM.git
   cd V-SLAM
   ```

2. **Install Required Packages**

   Make sure you have the necessary packages installed. You might need to install them using `pip` or `apt-get`.

   ```bash
   sudo apt-get update
   sudo apt-get install python3-pip ros-noetic-mavros ros-noetic-mavros-extras
   pip3 install -r requirements.txt
   ```

3. **Set Up MAVProxy**

   Install MAVProxy if you haven't already:

   ```bash
   pip3 install MAVProxy
   ```

## Configuration and Usage

### 1. Start MAVProxy

   Start MAVProxy with the following command to establish the communication channels:

   ```bash
   mavproxy.py --out udp:127.0.0.1:14540 --out udp:127.0.0.1:14550 --out 127.0.0.1:14560
   ```

### 2. Launch MAVROS

   Launch the MAVROS node with the following command:

   ```bash
   roslaunch mavros apm.launch
   ```

   Ensure that the `apm.launch` file is correctly set up for your MAVLink communication parameters.

### 3. Start the Camera Node

   Launch the RealSense camera node for visual input:

   ```bash
   roslaunch realsense2_camera opensource_tracking.launch
   ```

   Make sure that `opensource_tracking.launch` is configured according to your camera setup.

### 4. Run the Python Script

   Finally, run the Python script to start the Visual SLAM process:

   ```bash
   python3 all.py
   ```

## Troubleshooting

- **MAVProxy Connection Issues**: Ensure that MAVProxy is correctly configured and running. Check for any error messages in the terminal.
- **ROS Launch Errors**: Verify that all ROS packages are installed and sourced properly. Use `roscd` to ensure paths are correct.
- **Camera Not Detected**: Confirm that the RealSense camera is connected and recognized by your system. Check `dmesg` or `lsusb` for device detection.

## Contribution

Feel free to open issues or pull requests if you encounter problems or have improvements to suggest. Contributions are welcome!



## Contact

For questions or support, please contact [arjun222gupta@gmail.com](mailto:arjun222gupta@gmail.com).

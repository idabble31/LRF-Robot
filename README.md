# Logistics Robot Fleet

![image](https://github.com/user-attachments/assets/53bc3ca7-ded4-4cff-9a44-69e6e6c7b088)

## Overview

This repository hosts the ROS-based software for a logistics robot designed for autonomous navigation in indoor environments, such as warehouses or factories. The robot leverages a variety of sensors, including **RPLidar** for LiDAR-based mapping and obstacle detection, and **Intel RealSense** for depth sensing and visual navigation. 

In addition to sensor integration, the repository offers flexibility in motion planning with multiple global and local planning algorithms, allowing you to tailor the robot's navigation capabilities to your specific use case. These planning algorithms are made available through the integration of the [ros_motion_planning](https://github.com/ai-winter/ros_motion_planning) repository, providing advanced path planning and obstacle avoidance solutions.

## Features
- **Lidar-based obstacle detection** using `RPLidar A1`.
- **Sensor fusion** with `lrf_sensor_fusion` for enhanced perception.
- **Multiple Path planning algorithms to try** via `teb_local_planner` and `ros_motion_planning`.
- **Real-time control and data collection** using `lrf_control` and `lrf_data_collector`.
- **Intel RealSense integration** for visual navigation (`realsense-ros`).

## Repository Structure
```plaintext
logistics_robot_ws/
├── src/
│   ├── teb_local_planner/   # Elastic band local planner
│   ├── lrf_control/           # Control algorithms for the robot
│   ├── lrf_data_collector/    # Data collection tools
│   ├── ransel/                # Additional functionality (TBD)
│   ├── realsense-ros/         # Intel RealSense integration
│   ├── rplidar_ros/           # Lidar integration package
│   ├── ros_motion_planning/   # Motion planning algorithms
├── CMakeLists.txt             # Build configuration
└── README.md                  # Project documentation
```

## Requirements
- **ROS**: Make sure you have installed ROS (e.g., ROS Noetic or Melodic).
- **Dependencies**: 
  - `rplidar_ros` (change this according to the sensor you used) 
  - `realsense-ros` (change this according to the sensor you used)
  - `teb_local_planner`
  - `ros_motion_planning`

To install dependencies:

### 1. **Install ROS**

First, make sure you have ROS installed. The instructions below are for ROS Noetic, but you can adjust them for Melodic or other ROS versions.

For **ROS Noetic** on Ubuntu 20.04, use:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-noetic-desktop-full
```

Initialize `rosdep`:

```bash
sudo rosdep init
rosdep update
```

Add ROS setup to your `.bashrc` file:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install `catkin`:

```bash
sudo apt install python3-catkin-tools
```

### 2. **Install RPLidar ROS Package**

The `rplidar_ros` package is required for Lidar integration. You can install it as follows:

```bash
sudo apt install ros-noetic-rplidar-ros
```

Alternatively, you can clone and build the package manually:

```bash
cd ~/logistics_robot_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ..
catkin_make
```

### 3. **Install Intel RealSense ROS Package**

The `realsense-ros` package is required for depth sensing and visual navigation using Intel RealSense cameras. You can install it using the following steps:

#### 3.1 Install the required packages:

```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository ppa:ubuntu-sdk-team/ppa
sudo apt update
sudo apt install ros-noetic-realsense2-camera ros-noetic-realsense2-description
```

Alternatively, clone the repository:

```bash
cd ~/logistics_robot_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd ..
catkin_make
```

### 4. **Install teb_local_planner**

The `teb_local_planner` package is used for local path planning. To install:

```bash
sudo apt install ros-noetic-teb-local-planner
```

Or manually clone it into your workspace:

```bash
cd ~/logistics_robot_ws/src
git clone https://github.com/rst-tu-dortmund/teb_local_planner.git
cd ..
catkin_make
```

### 5. **Install ros_motion_planning Package**

The `ros_motion_planning` package provides additional motion planning functionality for your logistics robot. Install it using the following command:

```bash
sudo apt install ros-noetic-moveit
```

Alternatively, if you have a custom version of `ros_motion_planning`, clone it into your workspace:

```bash
cd ~/logistics_robot_ws/src
git clone https://github.com/your-username/ros_motion_planning.git
cd ..
catkin_make
```

### 6. **Install Additional Dependencies**

You may need additional libraries for things like OpenCV and PCL (Point Cloud Library). Install these as follows:

- **OpenCV** (for image processing):

```bash
sudo apt install libopencv-dev python3-opencv
```

- **PCL (Point Cloud Library)** (for 3D point cloud processing):

```bash
sudo apt install libpcl-dev
```

### 7. **Install Sensor Fusion Packages**

If you need to handle multiple sensor inputs, you can use packages like `robot_localization` for sensor fusion:

```bash
sudo apt install ros-noetic-robot-localization
```

## Building the Workspace

Once all dependencies are installed, build the workspace using:

```bash
cd ~/logistics_robot_ws
catkin_make
```

Make sure to source the workspace after building:

```bash
source devel/setup.bash
```

You can add this line to your `.bashrc` file for future convenience:

```bash
echo "source ~/logistics_robot_ws/devel/setup.bash" >> ~/.bashrc
```
```

# ros_gz_crazyflie

## Overview
The `ros_gz_crazyflie` repository contains several ROS 2 packages,
each designed for simulating the crazyflie robot in Gazebo Fortress.
These packages provide the necessary files and configurations to
spawn the crazyflie robot in the Gazebo simulation environment and
interact with it using ROS 2.
## Features
- Pre-configured launch files for easy simulation setup.
- Crazyflie SDF model integration.
- Compatibility with Gazebo Fortress.
- ROS 2 interface for controlling and monitoring the robot.
## Requirements
- **Operating System:** Ubuntu 22.04
- **ROS 2 Distribution:** Humble
- **Gazebo Version:** Fortress
## Installation
1. Clone the repository into your ROS 2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url>
```
2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build
```
3. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```
## Usage
### Launching the Simulation
#### Indoor Simulation
To launch the Crazyflie in an indoor environment, use the following
command:
```bash
ros2 launch ros_gz_crazyflie_bringup crazyflie_house.launch.py
```
#### Outdoor Simulation
To launch the Crazyflie in an outdoor environment, use the following
command:
```bash
ros2 launch ros_gz_crazyflie_bringup crazyflie_forest.launch.py
```
### Controlling the Robot
You can control the crazyflie using teleoperation
node. For example, to control the robot via the `/cmd_vel` topic:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Take off with pressing 't' and then control the crazyflie with the keyboard according to the teleop_twist_keyboard instructions.


## Crazyflie Mapping

To run RTAB-Map for mapping, use the following command:
```
ros2 launch ros_gz_crazyflie_nav rtab_mapping_house.launch.py
```
This launch file will open Gazebo Ignition, RViz, and RTAB-Map visualization tools (rtabviz). When the map construction is completed, exit the map construction node by pressing Ctrl+C. The system will automatically save the map.

The default saving path of the map is:
```
~/.ros/rtabmap.db
```
To view the map database, use the following command:
```
rtabmap-databaseViewer path/to/rtabmap.db
```
From there, you can save the map in .pcd format for further use. The .pcd file can be seen by downloading the extension `pcd-viewer` on vscode

## Troubleshooting
### Gazebo Fails to Launch
Ensure that your `IGN_GAZEBO_RESOURCE_PATH` environment variable is correctly set, for example: 
```bash
export IGN_GAZEBO_RESOURCE_PATH=~/ros2_ws/src/ros_gz_crazyflie/simulator_files/gazebo
```




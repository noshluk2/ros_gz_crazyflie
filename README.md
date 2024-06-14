# ros-gz-crazyflie

Made for ROS2 humble and Gazebo Fortress

## Usage

Build with

    colcon build --cmake-args -DBUILD_TESTING=ON

Run with

    colcon build --cmake-args -DBUILD_TESTING=ON

Velocity control with

    ros2 topic pub --once /crazyflie/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.1}, angular: {x: 0.0, y: 0.0, z: 0.0}}"


The sdf model Crazyflie is the adapted version of X3 UAV (found here on fuel: https://app.gazebosim.org/OpenRobotics/fuel/models/X3%20UAV). The visuals have been replaced by meshes made for the Crazyflie, to be found here: https://github.com/bitcraze/crazyflie-simulation.
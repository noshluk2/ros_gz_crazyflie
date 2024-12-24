# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_project_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_ign_gazebo')
    gz_model_path = os.getenv('IGN_GAZEBO_RESOURCE_PATH')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(gz_model_path, 'crazyflie', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_gazebo,
            'worlds',
            'forest_world.sdf -r'
        ])}.items(),
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_project_bringup, 'rviz', 'sensor_viz.rviz')],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],

        output='screen'
    )

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'crazyflie/odom'])

    control = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.5},
            {'robot_prefix': '/crazyflie'},
            {'incoming_twist_topic': '/cmd_vel'},
            {'max_ang_z_rate': 0.4},
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gz_sim,
        map_static_tf,
        bridge,
        rviz2_node,
        control        ])
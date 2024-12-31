import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Setup paths
    gz_model_path = os.getenv('IGN_GAZEBO_RESOURCE_PATH')
    if not gz_model_path:
        raise RuntimeError("IGN_GAZEBO_RESOURCE_PATH is not set. Please configure your environment.")

    sdf_file = os.path.join(gz_model_path, 'crazyflie', 'model.sdf')
    if not os.path.exists(sdf_file):
        raise FileNotFoundError(f"Model file not found: {sdf_file}")


    # RViz node
    crazyflie_bringup_path = get_package_share_directory('ros_gz_crazyflie_bringup')

    rviz_path = os.path.join(
        crazyflie_bringup_path, "rviz", "default.rviz"
    )

    if not os.path.exists(rviz_path):
        raise FileNotFoundError(f"RViz configuration file not found: {rviz_path}")

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Gazebo simulation
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={
            'gz_args': os.path.join(get_package_share_directory('ros_gz_crazyflie_gazebo'), 'worlds', 'crazyflie_world.sdf -r')
        }.items(),
    )

    # Bridge node
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('ros_gz_crazyflie_bringup'), 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],
        output='screen'
    )

    # Control node
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

    # Static Transform node (map -> odom)
    tf2_map = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            output="screen",
        )


    # Static Transform node (map -> odom)
    tf2_camera_rgb_frame = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "-1.5708", "0", "-1.5708", "camera_link", "camera_frame_rgb"],
            output="screen",
        )

    # Static Transform node (map -> odom)
    tf2_camera_link = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "crazyflie/body", "camera_link"],
            output="screen",
        )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_path],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        control,
        tf2_map,
        tf2_camera_link,
        tf2_camera_rgb_frame,
        rviz_node
    ])

    
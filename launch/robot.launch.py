"""Everything that runs on the real robot: URDF, lidar, motor bridge, EKF.

Run this on the Pi, then run mapping.launch.py or navigation.launch.py
(here or on your laptop over the same ROS_DOMAIN_ID).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory('casabot')
    urdf = os.path.join(pkg, 'urdf', 'casabot.urdf.xacro')

    lidar_port = LaunchConfiguration('lidar_port')
    base_port = LaunchConfiguration('base_port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='Serial port of the RPLIDAR'),
        DeclareLaunchArgument('base_port', default_value='/dev/ttyUSB1',
                              description='Serial port of the ESP32 base controller'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf, ' use_sim:=false']),
                'use_sim_time': use_sim_time,
            }],
        ),
        # The ESP32 reports encoder counts, so nothing else needs to guess joint angles.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='casabot',
            executable='base_driver',
            parameters=[{'port': base_port, 'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,   # A1/A2. C1 and S1 use 256000.
                'frame_id': 'lidar_link',
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
                'use_sim_time': use_sim_time,
            }],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[
                PathJoinSubstitution([FindPackageShare('casabot'), 'config', 'ekf.yaml']),
                {'use_sim_time': use_sim_time},
            ],
            output='screen',
        ),
    ])

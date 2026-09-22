"""Same robot in Gazebo Harmonic, so the whole stack is testable before any
hardware arrives. Replaces robot.launch.py; everything downstream is identical.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('casabot')
    urdf = os.path.join(pkg, 'urdf', 'casabot.urdf.xacro')
    world = LaunchConfiguration('world')

    robot_description = Command(['xacro ', urdf, ' use_sim:=true'])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v1 ', world], 'on_exit_shutdown': 'true'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'house.sdf')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),

        gz,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'casabot', '-x', '-2.0', '-y', '-1.5', '-z', '0.08'],
            output='screen',
        ),
        # Gazebo's own odom TF is deliberately not bridged: robot_localization
        # publishes odom -> base_footprint in sim exactly as it does on hardware.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml'), {'use_sim_time': True}],
            output='screen',
        ),
    ])

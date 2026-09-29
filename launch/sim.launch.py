"""Same robot in Gazebo Harmonic, so the whole stack is testable before any
hardware arrives. Replaces robot.launch.py; everything downstream is identical.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('casabot')
    urdf = os.path.join(pkg, 'urdf', 'casabot.urdf.xacro')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    worlds = os.path.join(pkg, 'worlds')
    # world:=flat picks worlds/flat.sdf; anything with a slash is used as a path.
    world_file = PythonExpression([
        "'", world, "' if '/' in '", world, "' else '", worlds, "/' + '", world, "' + '.sdf'"])

    robot_description = Command(['xacro ', urdf, ' use_sim:=true'])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        # gz_args is built here, so passing gz_args:= on the command line has no
        # effect. Use headless:=true to drop the GUI.
        launch_arguments={
            'gz_args': [PythonExpression(["'-s -r -v1 ' if '", headless, "' == 'true' else '-r -v1 '"]), world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='house',
                              description='house, flat, or a path to an .sdf file'),
        # Where the robot appears. Each bundled world has a spot in open floor.
        DeclareLaunchArgument('x', default_value=PythonExpression(
            ["{'house': '-2.0', 'flat': '6.0'}.get('", world, "', '0.0')"])),
        DeclareLaunchArgument('y', default_value=PythonExpression(
            ["{'house': '-1.5', 'flat': '4.1'}.get('", world, "', '0.0')"])),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server only, no GUI'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),

        gz,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        # Same as on hardware. Gazebo's JointStatePublisher system has no rate
        # limit and publishes every physics step (1 kHz), which the bridge,
        # robot_state_publisher and every TF listener then have to chew through,
        # all for two wheel angles nothing downstream uses.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'casabot',
                       '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.08'],
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

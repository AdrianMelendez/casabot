"""Autonomous mapping: slam_toolbox builds the map, Nav2 drives, and the
explorer picks where to go next until nothing is left to see.

    ros2 launch casabot sim.launch.py world:=flat        # terminal 1
    ros2 launch casabot explore.launch.py use_sim_time:=true

The finished map is written to ~/.casabot/map.{pgm,yaml}, which is what
navigation.launch.py loads by default.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg = get_package_share_directory('casabot')
    nav2 = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('map_path', default_value=os.path.expanduser('~/.casabot/map'),
                              description='Where to save the finished map (no extension)'),
        DeclareLaunchArgument('places_path', default_value=os.path.expanduser('~/.casabot/places.yaml'),
                              description='Where the rooms found are saved as places'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'mapping.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        # Nav2's lifecycle manager takes the whole stack down if a node misses
        # heartbeats for bond_timeout (4 s upstream), which a loaded machine or a
        # Raspberry Pi can do. navigation_launch.py hardcodes that node's
        # parameters, so it is set here for every node that follows.
        SetParameter(name='bond_timeout', value=10.0),
        # Nav2 without AMCL or map_server: slam_toolbox provides both the map and
        # the map -> odom transform while exploring.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(pkg, 'config', 'nav2.yaml'),
                'autostart': 'true',
            }.items(),
        ),
        Node(
            package='casabot',
            executable='explore',
            parameters=[{'use_sim_time': use_sim_time,
                         'map_path': LaunchConfiguration('map_path'),
                         'places_path': LaunchConfiguration('places_path')}],
            output='screen',
        ),
    ])

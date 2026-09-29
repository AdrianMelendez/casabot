"""Localise against a saved map and run Nav2, so `places go <name>` works."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    pkg = get_package_share_directory('casabot')
    nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/.casabot/map.yaml'),
                              description='Map saved during the mapping run'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # Nav2's lifecycle manager takes the whole stack down if a node misses
        # heartbeats for bond_timeout (4 s upstream), which a loaded machine or a
        # Raspberry Pi can do. navigation_launch.py hardcodes that node's
        # parameters, so it is set here for every node that follows.
        SetParameter(name='bond_timeout', value=10.0),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(pkg, 'config', 'nav2.yaml'),
                'autostart': 'true',
                'use_composition': 'True',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])

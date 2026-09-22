"""Drive the house once with slam_toolbox running, then save the map.

Drive it with, in another shell:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard

Save the result with:

    ros2 run nav2_map_server map_saver_cli -f ~/.casabot/map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('casabot')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),

        # slam_toolbox is a lifecycle node: it does nothing at all until it is
        # configured and activated. Upstream's launch file emits those
        # transitions, so include it rather than re-implementing them here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': os.path.join(pkg, 'config', 'slam.yaml'),
                'use_sim_time': use_sim_time,
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz', 'nav2_default_view.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz),
        ),
    ])

# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""RTAB-Map 3D LiDAR SLAM. Includable from wsl_sim or runnable against a rosbag."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('wsl_slam'), 'config', 'rtabmap.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        namespace='rtabmap',
        output='screen',
        parameters=[
            config,
            {'use_sim_time': use_sim_time,
             'database_path': LaunchConfiguration('database_path')},
        ],
        remappings=[('scan_cloud', '/lidar/points')],
        # Start every run with an empty map; the previous database is replaced.
        arguments=['--delete_db_on_start'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/wsl_rtabmap.db'),
            description='RTAB-Map database written during the run (map, graph, scans).'),
        rtabmap,
    ])

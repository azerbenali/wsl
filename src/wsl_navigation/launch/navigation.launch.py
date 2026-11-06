# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Nav2 navigation stack on the RTAB-Map map. Includable from wsl_sim.

    ros2 launch wsl_navigation navigation.launch.py [params_file:=...] [autostart:=false]

Starts the planner, controller, smoother, behaviour and velocity smoother
servers, bt_navigator, waypoint_follower and one lifecycle manager. There is no
AMCL and no map_server: RTAB-Map provides map -> odom and /rtabmap/map, and
nothing started here publishes TF.

Velocity chain: controller_server and behavior_server publish on cmd_vel_nav,
velocity_smoother limits it and publishes the final geometry_msgs/Twist on
/cmd_vel, which the ros_gz bridge forwards to MecanumDrive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Order matters: the lifecycle manager configures and activates in this order,
# and bt_navigator needs the servers whose actions it calls.
LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
]

# (package, executable, extra remappings). The name of each node is its executable.
NODES = [
    ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_smoother', 'smoother_server', []),
    ('nav2_planner', 'planner_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
    ('nav2_velocity_smoother', 'velocity_smoother',
     [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
    ('nav2_bt_navigator', 'bt_navigator', []),
    ('nav2_waypoint_follower', 'waypoint_follower', []),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    servers = [
        Node(package=package, executable=executable, name=executable, output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}],
             remappings=remappings)
        for package, executable, remappings in NODES
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'autostart': LaunchConfiguration('autostart'),
                     'node_names': LIFECYCLE_NODES}])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('wsl_navigation'), 'config', 'nav2.yaml'),
            description='Nav2 parameters for every node started here.'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate the Nav2 servers on startup.'),
        *servers,
        lifecycle_manager,
    ])

# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Run one benchmark scenario end to end and write results/<scenario>_<time>/.

    ros2 launch wsl_benchmark benchmark.launch.py scenario:=warehouse_loop

The scenario file (config/scenarios/<name>.yaml) fixes the spawn pose, the
robot/sensor arguments and the route, so a run is reproducible from the
repository alone. The launch shuts down when the recorder has written results.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wsl_benchmark.route import Scenario


def launch_setup(context):
    share = get_package_share_directory('wsl_benchmark')
    name = LaunchConfiguration('scenario').perform(context)
    path = os.path.join(share, 'config', 'scenarios', f'{name}.yaml')
    if not os.path.exists(path):
        available = sorted(f[:-5] for f in os.listdir(os.path.dirname(path)))
        raise RuntimeError(f'Unknown scenario "{name}". Available: {", ".join(available)}')
    scenario = Scenario.load(path)

    system_args = {
        'x': str(scenario.spawn['x']), 'y': str(scenario.spawn['y']),
        'yaw': str(scenario.spawn['yaw']),
        'ground_truth': 'true', 'slam': 'true', 'perception': 'true',
        'headless': LaunchConfiguration('headless').perform(context),
        'rviz': LaunchConfiguration('rviz').perform(context),
        **{k: str(v) for k, v in scenario.robot.items()},
    }
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wsl_sim'), 'launch', 'warehouse.launch.py')),
        launch_arguments=system_args.items())

    lidar_hz = float(scenario.robot.get('lidar_rate_hz', 10))
    camera_hz = float(scenario.robot.get('camera_rate_hz', 20))
    recorder = Node(
        package='wsl_benchmark', executable='recorder', output='screen',
        parameters=[{
            'use_sim_time': True,
            'scenario': scenario.name,
            'output_dir': LaunchConfiguration('output_dir').perform(context),
            'expected_rates_hz': [f'/lidar/points:{lidar_hz}', f'/camera/image_raw:{camera_hz}',
                                  f'/camera/camera_info:{camera_hz}', '/imu/data:100.0',
                                  '/odom:50.0'],
        }])
    actions = [system, recorder,
               RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=[Shutdown(
                   reason='benchmark results written')]))]

    if scenario.waypoints:
        follower = Node(
            package='wsl_benchmark', executable='route_follower', output='screen',
            parameters=[{'use_sim_time': True, 'scenario_file': path}])
        # The recorder ends the run once it has the route outcome; this timer
        # only matters if the follower dies without publishing one.
        actions += [follower, RegisterEventHandler(OnProcessExit(
            target_action=follower, on_exit=[TimerAction(period=15.0, actions=[Shutdown(
                reason='route follower exited')])]))]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='warehouse_loop',
                              description='Scenario in wsl_benchmark/config/scenarios.'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(os.getcwd(), 'results'),
                              description='Parent directory of the run directory.'),
        OpaqueFunction(function=launch_setup),
    ])

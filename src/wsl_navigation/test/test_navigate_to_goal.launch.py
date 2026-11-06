# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
End-to-end navigation test: the real warehouse launch with SLAM and Nav2, headless.

Waits for Nav2 to become active, checks that the velocity chain ends in one
unstamped Twist publisher on /cmd_vel, that Nav2 adds no TF publisher and that
the global costmap receives the RTAB-Map map, then sends a NavigateToPose goal
around a pillar and checks the outcome and the final map -> base_footprint pose.

Requires an OpenGL-capable renderer (Mesa llvmpipe works), hence opt-in via
-DWSL_SIM_TESTS=ON. wsl_sim depends on this package, so it cannot be a declared
test dependency: source install/setup.bash before colcon test.
"""

import importlib.util
import math
import os
import time
import unittest

import pytest
import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_ros

WAREHOUSE_LAUNCH = os.path.join(
    get_package_share_directory('wsl_sim'), 'launch', 'warehouse.launch.py')
NAV2_CONFIG = os.path.join(get_package_share_directory('wsl_navigation'), 'config', 'nav2.yaml')

# Goal in the world frame of worlds/warehouse.sdf: open staging area, 1.6 m from
# the north wall and 1.7 m from pallet stack stage_2. The straight line from the
# spawn point runs through pillar_n_west at (-10, 11.5), so the planner has to
# route around it. Facing north.
GOAL_WORLD = (-8.5, 13.4, math.pi / 2)
# Only these nodes may publish /tf: odometry (bridged), wheels and SLAM.
TF_PUBLISHERS = {'gz_bridge', 'robot_state_publisher', 'rtabmap'}


@pytest.mark.launch_test
def generate_test_description():
    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(WAREHOUSE_LAUNCH),
        launch_arguments={'headless': 'true', 'rviz': 'false', 'nav': 'true'}.items())
    return LaunchDescription([system, launch_testing.actions.ReadyToTest()])


def spawn_pose():
    """Default spawn pose of warehouse.launch.py; the map frame starts there."""
    spec = importlib.util.spec_from_file_location('warehouse_launch', WAREHOUSE_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    defaults = {name: float(default) for name, default, _ in module.ARGUMENTS
                if name in ('x', 'y', 'yaw')}
    return defaults['x'], defaults['y'], defaults['yaw']


def goal_in_map():
    sx, sy, syaw = spawn_pose()
    gx, gy, gyaw = GOAL_WORLD
    dx, dy = gx - sx, gy - sy
    return (math.cos(syaw) * dx + math.sin(syaw) * dy,
            -math.sin(syaw) * dx + math.cos(syaw) * dy,
            gyaw - syaw)


def goal_tolerances():
    with open(NAV2_CONFIG) as f:
        controller = yaml.safe_load(f)['controller_server']['ros__parameters']
    checker = controller[controller['goal_checker_plugins'][0]]
    return checker['xy_goal_tolerance'], checker['yaw_goal_tolerance']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class TestNavigateToGoal(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('navigate_to_goal_test', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        cls.navigate = ActionClient(cls.node, NavigateToPose, 'navigate_to_pose')
        cls.costmaps = []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cls.node.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                     cls.costmaps.append, latched)

    @classmethod
    def tearDownClass(cls):
        cls.navigate.destroy()
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return
        self.fail(f'timed out after {timeout_s} s waiting for {what}')

    def wait_for(self, future, timeout_s, what):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_s)
        self.assertTrue(future.done(), f'timed out after {timeout_s} s waiting for {what}')
        return future.result()

    def test_1_nav2_becomes_active(self):
        is_active = self.node.create_client(Trigger, '/lifecycle_manager_navigation/is_active')

        def active():
            if not is_active.service_is_ready():
                return False
            return self.wait_for(is_active.call_async(Trigger.Request()), 10.0,
                                 'is_active').success

        # Startup includes the Gazebo server boot, spawning and Nav2 activation,
        # which waits for map -> base_footprint from RTAB-Map and odometry.
        self.spin_until(active, 300.0, 'Nav2 lifecycle nodes to become active')
        self.spin_until(
            lambda: self.tf_buffer.can_transform('map', 'base_footprint', Time()),
            60.0, 'map -> base_footprint')
        self.assertTrue(self.navigate.wait_for_server(timeout_sec=30.0))

    def test_2_interfaces(self):
        cmd_vel = self.node.get_publishers_info_by_topic('/cmd_vel')
        self.assertEqual({p.node_name for p in cmd_vel}, {'velocity_smoother'})
        self.assertEqual({p.topic_type for p in cmd_vel}, {'geometry_msgs/msg/Twist'})
        tf = {p.node_name for p in self.node.get_publishers_info_by_topic('/tf')}
        self.assertLessEqual(tf, TF_PUBLISHERS, 'a node other than odometry, '
                             'robot_state_publisher and RTAB-Map publishes TF')
        self.assertIn('rtabmap', tf)
        # The static layer must be fed by /rtabmap/map: known cells, not all unknown.
        self.spin_until(lambda: any(0 <= c < 100 for c in self.costmaps[-1].data)
                        if self.costmaps else False, 60.0, 'free cells in the global costmap')

    def test_3_navigate_to_goal(self):
        # RTAB-Map starts its map at the spawn pose: a robot elsewhere means this
        # test is talking to another simulation (e.g. a leftover Gazebo server).
        start = self.tf_buffer.lookup_transform('map', 'base_footprint', Time()).transform
        self.assertLess(math.hypot(start.translation.x, start.translation.y), 0.1,
                        'robot does not start at the map origin (spawn pose)')

        x, y, yaw = goal_in_map()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        handle = self.wait_for(self.navigate.send_goal_async(goal), 30.0, 'goal response')
        self.assertTrue(handle.accepted, 'NavigateToPose goal rejected')
        result = self.wait_for(handle.get_result_async(), 300.0, 'NavigateToPose result')
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED,
                         f'navigation ended with status {result.status}, '
                         f'error {result.result.error_code}')

        # The pose the goal checker accepted, before anything else can move it.
        pose = self.tf_buffer.lookup_transform('map', 'base_footprint', Time()).transform
        xy_tolerance, yaw_tolerance = goal_tolerances()
        distance = math.hypot(pose.translation.x - x, pose.translation.y - y)
        heading = abs(angle_diff(yaw_of(pose.rotation), yaw))
        self.assertLessEqual(distance, xy_tolerance,
                             f'stopped {distance:.3f} m from the goal')
        self.assertLessEqual(heading, yaw_tolerance,
                             f'stopped {heading:.3f} rad off the goal heading')

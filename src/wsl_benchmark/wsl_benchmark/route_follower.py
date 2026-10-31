# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Experiment harness: drives the robot along a scenario route.

It steers on ground truth on purpose. The aim is to reproduce the same
physical path in every run (a perfect human driver), so that SLAM
configurations are compared on identical input. It publishes only /cmd_vel;
nothing it computes reaches any estimator.

Outcome (completed / stuck / timeout) is published latched on
/benchmark/route_status. Exit code 0 means the route was completed.
"""

import math
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros

from wsl_benchmark.route import densify, pursuit_command, PursuitConfig, PursuitState, Scenario

STUCK_WINDOW_S = 30.0
STUCK_MIN_PROGRESS_M = 0.5


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RouteFollower(Node):

    def __init__(self):
        super().__init__('route_follower')
        scenario = Scenario.load(self.declare_parameter('scenario_file', '').value)
        if not scenario.waypoints:
            raise ValueError(f'scenario {scenario.name} has no route')
        self.start_delay = Duration(seconds=self.declare_parameter('start_delay_s', 3.0).value)
        self.map_frame_timeout_s = self.declare_parameter('map_frame_timeout_s', 60.0).value
        self.cfg = PursuitConfig(speed_mps=scenario.speed_mps)
        self.path = densify(scenario.waypoints, self.cfg.spacing_m)
        self.max_duration = Duration(seconds=scenario.max_duration_s)
        self.state = PursuitState()
        self.pose = None
        self.first_pose_time = None
        self.start_time = None
        self.outcome = None
        self.progress_marks = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/benchmark/route_status', latched)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_ground_truth, 10)
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info(
            f'Route "{scenario.name}": {len(scenario.waypoints)} waypoints, '
            f'{(len(self.path) - 1) * self.cfg.spacing_m:.1f} m at {scenario.speed_mps} m/s')

    def on_ground_truth(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if self.first_pose_time is None:
            self.first_pose_time = Time.from_msg(msg.header.stamp)

    def ready(self, now):
        if self.pose is None or now - self.first_pose_time < self.start_delay:
            return False
        # Give SLAM its first node before moving, but do not depend on it.
        if self.tf_buffer.can_transform('map', 'base_footprint', Time()):
            return True
        waited = (now - self.first_pose_time).nanoseconds * 1e-9
        if waited > self.map_frame_timeout_s:
            self.get_logger().warn(f'No map frame after {waited:.0f} s; starting without SLAM')
            return True
        return False

    def finish(self, outcome):
        self.cmd_pub.publish(Twist())
        self.outcome = outcome
        self.status_pub.publish(String(data=outcome))
        log = self.get_logger().info if outcome == 'completed' else self.get_logger().error
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        log(f'Route {outcome} after {elapsed_s:.1f} s '
            f'(sim), progress {self.state.progress * self.cfg.spacing_m:.1f} m')
        self.timer.cancel()

    def step(self):
        now = self.get_clock().now()
        if self.start_time is None:
            if not self.ready(now):
                return
            self.start_time = now
            self.get_logger().info('Starting route')

        elapsed = now - self.start_time
        progress_m = self.state.progress * self.cfg.spacing_m
        self.progress_marks.append((elapsed.nanoseconds * 1e-9, progress_m))
        window_start = elapsed.nanoseconds * 1e-9 - STUCK_WINDOW_S
        old = [m for t, m in self.progress_marks if t <= window_start]
        if old and progress_m - old[-1] < STUCK_MIN_PROGRESS_M:
            self.finish('stuck')
            return
        self.progress_marks = [(t, m) for t, m in self.progress_marks if t > window_start - 1.0]
        if elapsed > self.max_duration:
            self.finish('timeout')
            return

        v, w = pursuit_command(*self.pose, self.path, self.state, self.cfg)
        if self.state.done:
            self.finish('completed')
            return
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = None
    try:
        node = RouteFollower()
        while rclpy.ok() and node.outcome is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Let the latched status reach late subscribers before exiting.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        outcome = node.outcome if node else None
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0 if outcome == 'completed' else 1)

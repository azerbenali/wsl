# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Passive benchmark recorder.

Observes the running system and writes raw observations to a run directory;
evaluate.py derives every metric from them. The recorder only subscribes,
except for TF lookups, so it cannot influence the autonomy pipeline. Its own
CPU cost is visible in resources.csv like any other component.

Finishes when /benchmark/route_status reports an outcome, or on shutdown.
"""

import csv
from datetime import datetime
import json
import math
import os
import time

from diagnostic_msgs.msg import DiagnosticArray
import psutil
import rclpy
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMsg
from rtabmap_msgs.msg import Info, MapData
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from std_msgs.msg import String
import tf2_ros
from vision_msgs.msg import Detection3DArray

from wsl_benchmark.evaluate import evaluate_run

# Executable name (psutil "name", truncated to 15 characters by Linux) -> component.
COMPONENTS = {
    'rtabmap': 'slam',
    'perception_node': 'perception',
    'parameter_bridge': 'bridge',
    'parameter_bridg': 'bridge',
    'robot_state_publisher': 'robot_state_publisher',
    'robot_state_pub': 'robot_state_publisher',
    'rviz2': 'rviz',
}
CDR_LITTLE_ENDIAN = b'\x00\x01'
SENSOR_TYPES = {
    '/lidar/points': PointCloud2,
    '/camera/image_raw': Image,
    '/camera/camera_info': CameraInfo,
    '/imu/data': Imu,
    '/odom': Odometry,
}


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """SE(2) composition a * b of (x, y, yaw) tuples."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])))


def component_of(proc):
    name = proc.info['name'] or ''
    if name in COMPONENTS:
        return COMPONENTS[name]
    cmdline = ' '.join(proc.info['cmdline'] or [])
    # `gz sim` runs the server inside the Ruby interpreter of the gz CLI.
    if name in ('ruby', 'gz') and ' sim' in cmdline:
        return 'gazebo'
    if 'wsl_benchmark' in cmdline and 'recorder' in cmdline:
        return 'benchmark_recorder'
    return None


class BenchmarkRecorder(Node):

    def __init__(self):
        super().__init__('benchmark_recorder')
        self.scenario = self.declare_parameter('scenario', 'manual').value
        output_root = os.path.expanduser(self.declare_parameter('output_dir', 'results').value)
        expected = self.declare_parameter(
            'expected_rates_hz', ['/lidar/points:10', '/camera/image_raw:20',
                                  '/camera/camera_info:20', '/imu/data:100', '/odom:50']).value
        self.expected_rates = {item.rsplit(':', 1)[0]: float(item.rsplit(':', 1)[1])
                               for item in expected}
        self.run_dir = os.path.join(
            output_root, f'{self.scenario}_{datetime.now().strftime("%Y%m%d_%H%M%S")}')
        os.makedirs(self.run_dir, exist_ok=True)

        self.sensor_rows = []
        self.perception_rows = []
        self.cluster_rows = []
        self.diagnostic_rows = []
        self.raw_cloud_arrivals = {}
        self.trajectories = {'ground_truth': [], 'odometry': [], 'slam_online': []}
        self.node_stamps = {}
        self.graph = None
        self.slam_rows = []
        self.resource_rows = []
        self.clock = {}
        self.route_outcome = None
        self.finished = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Reliable subscriptions measure what the producers emit; a best-effort
        # subscriber would also count DDS losses of large messages.
        for topic in self.expected_rates:
            if topic == '/lidar/points':
                self.create_subscription(PointCloud2, topic, self.on_raw_cloud, 20)
            elif topic == '/odom':
                self.create_subscription(Odometry, topic, self.on_odom, 50)
            else:
                # Raw (serialized) subscription: only the header stamp is needed,
                # and deserializing 920 KB images in Python would load the system.
                self.create_subscription(
                    SENSOR_TYPES[topic], topic, self.make_stamp_recorder(topic), 50, raw=True)
        self.create_subscription(PointCloud2, '/perception/filtered_points', self.on_filtered, 20)
        self.create_subscription(Detection3DArray, '/perception/clusters', self.on_clusters, 20)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diagnostics, 20)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_ground_truth, 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # mapData carries the optimised graph and each new node's capture stamp.
        # (mapPath is not usable: all its poses carry the path's publish stamp.)
        self.create_subscription(MapData, '/rtabmap/mapData', self.on_map_data, 10)
        self.create_subscription(Info, '/rtabmap/info', self.on_slam_info, 20)
        self.create_subscription(ClockMsg, '/clock', self.on_clock, 10)
        self.create_subscription(String, '/benchmark/route_status', self.on_route_status, latched)

        self.processes = {}
        self.create_timer(1.0, self.sample_resources, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(f'Recording scenario "{self.scenario}" to {self.run_dir}')

    def make_stamp_recorder(self, topic):
        msg_type = SENSOR_TYPES[topic]

        def record(serialized):
            arrival = time.monotonic()
            if serialized[:2] == CDR_LITTLE_ENDIAN:
                # Every message here starts with std_msgs/Header, so after the
                # 4-byte encapsulation come stamp.sec (int32), stamp.nanosec (uint32).
                t = (int.from_bytes(serialized[4:8], 'little', signed=True) +
                     int.from_bytes(serialized[8:12], 'little') * 1e-9)
            else:
                t = stamp_s(deserialize_message(serialized, msg_type).header.stamp)
            self.sensor_rows.append((topic, t, arrival))
        return record

    def on_raw_cloud(self, msg):
        arrival = time.monotonic()
        t = stamp_s(msg.header.stamp)
        self.sensor_rows.append(('/lidar/points', t, arrival))
        self.raw_cloud_arrivals[t] = (arrival, msg.width * msg.height)
        if len(self.raw_cloud_arrivals) > 100:
            del self.raw_cloud_arrivals[min(self.raw_cloud_arrivals)]

    def on_filtered(self, msg):
        arrival = time.monotonic()
        t = stamp_s(msg.header.stamp)
        raw = self.raw_cloud_arrivals.get(t)
        # Perception keeps the input stamp, so arrival-time difference is the
        # end-to-end latency: bridge -> perception -> serialisation -> here.
        latency_ms = (arrival - raw[0]) * 1000.0 if raw else ''
        self.perception_rows.append((t, raw[1] if raw else '', msg.width * msg.height, latency_ms))

    def on_clusters(self, msg):
        self.cluster_rows.append((stamp_s(msg.header.stamp), len(msg.detections)))

    def on_diagnostics(self, msg):
        # Processing time measured inside perception_node, free of transport.
        for status in msg.status:
            if status.name.endswith('point cloud pipeline'):
                values = {kv.key: kv.value for kv in status.values}
                self.diagnostic_rows.append((
                    stamp_s(msg.header.stamp), values.get('clouds processed', '0'),
                    values.get('clouds rejected', '0'), values.get('processing mean [ms]', '0'),
                    values.get('processing max [ms]', '0')))

    def on_ground_truth(self, msg):
        p = msg.pose.pose
        self.trajectories['ground_truth'].append(
            (stamp_s(msg.header.stamp), p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_odom(self, msg):
        t = stamp_s(msg.header.stamp)
        self.sensor_rows.append(('/odom', t, time.monotonic()))
        p = msg.pose.pose
        odom_pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.trajectories['odometry'].append((t, *odom_pose))
        # Online SLAM estimate: the latest map -> odom correction applied to
        # this odometry sample, i.e. the pose a consumer would use right now.
        try:
            tf = self.tf_buffer.lookup_transform('map', 'odom', Time())
        except tf2_ros.TransformException:
            return
        map_odom = (tf.transform.translation.x, tf.transform.translation.y,
                    yaw_of(tf.transform.rotation))
        self.trajectories['slam_online'].append((t, *compose(map_odom, odom_pose)))

    def on_map_data(self, msg):
        for node in msg.nodes:
            self.node_stamps[node.id] = node.stamp
        self.graph = msg.graph

    def on_slam_info(self, msg):
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        self.slam_rows.append((stamp_s(msg.header.stamp), msg.loop_closure_id,
                               msg.proximity_detection_id, stats.get('Timing/Total/ms', '')))

    def on_clock(self, msg):
        sim, wall = stamp_s(msg.clock), time.monotonic()
        if 'sim_start' not in self.clock:
            self.clock.update(sim_start=sim, wall_start=wall)
        self.clock.update(sim_end=sim, wall_end=wall)

    def on_route_status(self, msg):
        self.route_outcome = msg.data
        self.get_logger().info(f'Route finished: {msg.data}')
        self.finished = True

    def sample_resources(self):
        now = time.monotonic()
        seen = set()
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            component = component_of(proc)
            if component is None:
                continue
            seen.add(proc.pid)
            try:
                if proc.pid not in self.processes:
                    self.processes[proc.pid] = proc
                    proc.cpu_percent(None)  # first call only primes the counter
                    continue
                self.resource_rows.append((now, component, proc.pid,
                                           self.processes[proc.pid].cpu_percent(None),
                                           proc.memory_info().rss / 2**20))
            except psutil.Error:
                pass
        self.processes = {pid: p for pid, p in self.processes.items() if pid in seen}

    def write(self):
        def dump(name, header, rows):
            with open(os.path.join(self.run_dir, name), 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(rows)

        for name, rows in self.trajectories.items():
            dump(f'{name}.csv', ('t', 'x', 'y', 'yaw'), rows)
        optimized = []
        if self.graph is not None:
            optimized = sorted(
                (self.node_stamps[i], p.position.x, p.position.y, yaw_of(p.orientation))
                for i, p in zip(self.graph.poses_id, self.graph.poses) if i in self.node_stamps)
        dump('slam_optimized.csv', ('t', 'x', 'y', 'yaw'), optimized)
        dump('sensor_stamps.csv', ('topic', 'stamp', 'arrival_wall'), self.sensor_rows)
        dump('perception.csv', ('stamp', 'raw_points', 'filtered_points', 'latency_ms'),
             self.perception_rows)
        dump('clusters.csv', ('stamp', 'clusters'), self.cluster_rows)
        dump('perception_diagnostics.csv', ('stamp', 'clouds', 'rejected', 'mean_ms', 'max_ms'),
             self.diagnostic_rows)
        dump('slam_info.csv', ('stamp', 'loop_closure_id', 'proximity_detection_id', 'total_ms'),
             self.slam_rows)
        dump('resources.csv', ('wall', 'component', 'pid', 'cpu_percent', 'rss_mb'),
             self.resource_rows)
        with open(os.path.join(self.run_dir, 'run.json'), 'w') as f:
            json.dump({'scenario': self.scenario, 'route_outcome': self.route_outcome,
                       'expected_rates_hz': self.expected_rates, 'clock': self.clock,
                       'cpu_count': psutil.cpu_count()}, f, indent=2)

        metrics = evaluate_run(self.run_dir)
        with open(os.path.join(self.run_dir, 'metrics.json'), 'w') as f:
            json.dump(metrics, f, indent=2)
        return metrics


def main():
    rclpy.init()
    node = BenchmarkRecorder()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Collect the final optimised graph published after the last update.
        deadline = time.monotonic() + 3.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        metrics = node.write()
        summary = metrics.get('trajectory_error', {}).get('slam_optimized') or {}
        node.get_logger().info(
            f'Wrote {node.run_dir} (route: {node.route_outcome}, '
            f'SLAM ATE RMSE: {summary.get("ate", {}).get("translation_m", {}).get("rmse")})')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

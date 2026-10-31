# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""evaluate_run on a synthetic run directory with known answers."""

import csv
import json
import math

import pytest

from wsl_benchmark.evaluate import evaluate_run


def write(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def test_evaluate_run(tmp_path):
    gt = [(i * 0.02, i * 0.01, 0.0, 0.0) for i in range(1001)]          # 10 m at 0.5 m/s
    odom = [(t, x * 1.01, 0.0, 0.0) for t, x, _, _ in gt[::2]]            # 1% scale error
    write(tmp_path / 'ground_truth.csv', ('t', 'x', 'y', 'yaw'), gt)
    write(tmp_path / 'odometry.csv', ('t', 'x', 'y', 'yaw'), odom)
    write(tmp_path / 'slam_online.csv', ('t', 'x', 'y', 'yaw'), [])
    write(tmp_path / 'slam_optimized.csv', ('t', 'x', 'y', 'yaw'), gt[::50])
    write(tmp_path / 'sensor_stamps.csv', ('topic', 'stamp', 'arrival_wall'),
          [('/lidar/points', i * 0.1, i * 0.2) for i in range(201)])
    write(tmp_path / 'perception.csv', ('stamp', 'raw_points', 'filtered_points', 'latency_ms'),
          [(i * 0.1, 1000, 250, 5.0) for i in range(10)] + [(1.0, '', 300, '')])
    write(tmp_path / 'perception_diagnostics.csv',
          ('stamp', 'clouds', 'rejected', 'mean_ms', 'max_ms'),
          [(1.0, 10, 0, 4.0, 6.0), (2.0, 30, 2, 2.0, 9.0)])
    write(tmp_path / 'clusters.csv', ('stamp', 'clusters'), [(0.0, 3), (0.1, 5)])
    write(tmp_path / 'slam_info.csv',
          ('stamp', 'loop_closure_id', 'proximity_detection_id', 'total_ms'),
          [(1.0, 0, 0, 50.0), (2.0, 0, 7, 70.0), (3.0, 3, 0, 60.0)])
    write(tmp_path / 'resources.csv', ('wall', 'component', 'pid', 'cpu_percent', 'rss_mb'),
          [(float(t), 'slam', 1, 50.0, 100.0 + t) for t in range(60)])
    (tmp_path / 'run.json').write_text(json.dumps({
        'scenario': 'synthetic', 'route_outcome': 'completed',
        'expected_rates_hz': {'/lidar/points': 10.0, '/imu/data': 100.0},
        'clock': {'sim_start': 0.0, 'sim_end': 20.0, 'wall_start': 0.0, 'wall_end': 40.0}}))

    m = evaluate_run(str(tmp_path))
    assert m['real_time_factor'] == pytest.approx(0.5)
    assert m['sensors']['/lidar/points']['rate_hz'] == pytest.approx(10.0)
    assert m['sensors']['/imu/data'] == {'count': 0}  # expected but never received
    assert m['perception']['reduction_percent'] == pytest.approx(75.0)
    assert m['perception']['unmatched_outputs'] == 1
    assert m['perception']['end_to_end_latency_ms']['count'] == 10
    in_node = m['perception']['in_node_processing_ms']
    assert in_node['mean'] == pytest.approx(2.5)   # cloud-weighted: (10*4 + 30*2) / 40
    assert in_node['max'] == 9.0
    assert in_node['rejected'] == 2
    assert m['perception']['clusters']['mean'] == pytest.approx(4.0)
    assert m['slam']['loop_closures'] == 1
    assert m['slam']['proximity_detections'] == 1
    assert m['slam']['first_closure_stamp'] == pytest.approx(2.0)
    assert 'slam_online' not in m['trajectory_error']
    odom = m['trajectory_error']['odometry']
    assert odom['drift']['final_translation_m'] == pytest.approx(0.1, rel=1e-6)
    assert odom['rpe']['translation_m']['mean'] == pytest.approx(0.01, rel=1e-3)
    assert m['trajectory_error']['slam_optimized']['ate']['translation_m']['rmse'] == \
        pytest.approx(0.0, abs=1e-9)
    slam_cpu = m['resources']['slam']
    assert slam_cpu['cpu_percent']['mean'] == pytest.approx(50.0)
    assert slam_cpu['rss_mb_growth'] == pytest.approx(39.0)   # samples 20..59 after warm-up
    assert math.isfinite(m['duration_wall_s'])

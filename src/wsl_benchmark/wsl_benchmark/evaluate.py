# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Turn a recorded run directory into metrics.json.

The recorder stores raw observations only; every number in metrics.json is
derived here, so evaluation can be changed and re-run without re-simulating:

    ros2 run wsl_benchmark evaluate results/<run>
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

from wsl_benchmark.metrics import evaluate_trajectory, period_stats, summarise, Trajectory

TRAJECTORIES = ('odometry', 'slam_online', 'slam_optimized')


def read_csv(path):
    if not os.path.exists(path):
        return []
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def read_trajectory(run_dir, name):
    rows = read_csv(os.path.join(run_dir, f'{name}.csv'))
    return Trajectory.from_rows([(float(r['t']), float(r['x']), float(r['y']), float(r['yaw']))
                                 for r in rows])


def sensor_metrics(run_dir, expected_rates):
    stamps = {}
    for row in read_csv(os.path.join(run_dir, 'sensor_stamps.csv')):
        stamps.setdefault(row['topic'], []).append(float(row['stamp']))
    return {topic: period_stats(stamps.get(topic, []), expected_rates.get(topic))
            for topic in sorted(set(stamps) | set(expected_rates))}


def perception_metrics(run_dir):
    rows = read_csv(os.path.join(run_dir, 'perception.csv'))
    if not rows:
        return None
    # An output whose input the recorder has not seen yet (callback ordering in
    # its executor) has no latency; it is counted rather than silently dropped.
    matched = [r for r in rows if r['raw_points']]
    raw = np.array([float(r['raw_points']) for r in matched])
    filtered = np.array([float(r['filtered_points']) for r in matched])
    clusters = [int(r['clusters']) for r in read_csv(os.path.join(run_dir, 'clusters.csv'))]
    diagnostics = read_csv(os.path.join(run_dir, 'perception_diagnostics.csv'))
    processed = sum(int(d['clouds']) for d in diagnostics)
    in_node = {}
    if processed:
        in_node = {
            'mean': sum(float(d['mean_ms']) * int(d['clouds']) for d in diagnostics) / processed,
            'max': max(float(d['max_ms']) for d in diagnostics),
            'clouds': processed,
            'rejected': sum(int(d['rejected']) for d in diagnostics),
        }
    return {
        'clouds': len(rows),
        'unmatched_outputs': len(rows) - len(matched),
        'raw_points': summarise(raw),
        'filtered_points': summarise(filtered),
        'reduction_percent':
            float(100.0 * (1.0 - filtered.sum() / raw.sum())) if raw.sum() else None,
        'end_to_end_latency_ms': summarise([float(r['latency_ms']) for r in matched]),
        'in_node_processing_ms': in_node,
        'clusters': summarise(clusters),
    }


def slam_metrics(run_dir):
    rows = read_csv(os.path.join(run_dir, 'slam_info.csv'))
    if not rows:
        return None
    loops = [r for r in rows if int(r['loop_closure_id']) > 0]
    proximity = [r for r in rows if int(r['proximity_detection_id']) > 0]
    return {
        'updates': len(rows),
        'loop_closures': len(loops),
        'proximity_detections': len(proximity),
        'first_closure_stamp': min((float(r['stamp']) for r in loops + proximity), default=None),
        'processing_ms': summarise([float(r['total_ms']) for r in rows if r['total_ms']]),
    }


def resource_metrics(run_dir, warmup_s=20.0):
    """Per-component CPU (% of one core) and resident memory, after a warm-up."""
    by_component = {}
    for row in read_csv(os.path.join(run_dir, 'resources.csv')):
        by_component.setdefault(row['component'], []).append(
            (float(row['wall']), float(row['cpu_percent']), float(row['rss_mb'])))
    result = {}
    for component, samples in sorted(by_component.items()):
        t0 = samples[0][0]
        steady = [s for s in samples if s[0] - t0 >= warmup_s] or samples
        rss = [s[2] for s in steady]
        result[component] = {
            'cpu_percent': summarise([s[1] for s in steady]),
            'rss_mb_max': max(rss),
            # Growth over the steady window, as a first check for unbounded allocation.
            'rss_mb_growth': rss[-1] - rss[0],
        }
    return result


def evaluate_run(run_dir):
    with open(os.path.join(run_dir, 'run.json')) as f:
        run = json.load(f)
    gt = read_trajectory(run_dir, 'ground_truth')
    trajectories = {}
    for name in TRAJECTORIES:
        est = read_trajectory(run_dir, name)
        if len(gt) > 2 and len(est) > 2:
            trajectories[name] = evaluate_trajectory(est, gt)

    clock = run.get('clock', {})
    sim_s = clock.get('sim_end', 0.0) - clock.get('sim_start', 0.0)
    wall_s = clock.get('wall_end', 0.0) - clock.get('wall_start', 0.0)
    return {
        'scenario': run.get('scenario'),
        'route_outcome': run.get('route_outcome'),
        'duration_sim_s': sim_s,
        'duration_wall_s': wall_s,
        'real_time_factor': sim_s / wall_s if wall_s > 0 else None,
        'sensors': sensor_metrics(run_dir, run.get('expected_rates_hz', {})),
        'perception': perception_metrics(run_dir),
        'slam': slam_metrics(run_dir),
        'trajectory_error': trajectories,
        'resources': resource_metrics(run_dir),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('run_dir')
    args = parser.parse_args(argv)
    if not os.path.exists(os.path.join(args.run_dir, 'run.json')):
        print(f'{args.run_dir}: not a recorded run (run.json missing)', file=sys.stderr)
        return 1
    metrics = evaluate_run(args.run_dir)
    with open(os.path.join(args.run_dir, 'metrics.json'), 'w') as f:
        json.dump(metrics, f, indent=2)
    print(json.dumps(metrics, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())

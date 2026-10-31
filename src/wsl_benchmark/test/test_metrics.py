# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Benchmark mathematics checked against analytically known results."""

import math

import numpy as np
import pytest

from wsl_benchmark.metrics import (absolute_trajectory_error, associate, drift, interpolate,
                                   period_stats, relative_pose_error, summarise,
                                   trajectory_length, Trajectory, umeyama, wrap_angle)


def line(n=101, length=10.0, dt=0.1):
    x = np.linspace(0.0, length, n)
    return Trajectory(np.arange(n) * dt, x, np.zeros(n), np.zeros(n))


def transform(traj, theta, tx, ty):
    c, s = math.cos(theta), math.sin(theta)
    return Trajectory(traj.t, c * traj.x - s * traj.y + tx, s * traj.x + c * traj.y + ty,
                      wrap_angle(traj.yaw + theta))


def test_period_stats_uniform_stream():
    stats = period_stats(np.arange(0, 10.0001, 0.1), expected_rate_hz=10.0)
    assert stats['count'] == 101
    assert stats['rate_hz'] == pytest.approx(10.0)
    assert stats['period_mean_s'] == pytest.approx(0.1)
    assert stats['period_std_s'] == pytest.approx(0.0, abs=1e-12)
    assert stats['stalls'] == 0
    assert stats['timestamp_regressions'] == 0
    assert stats['rate_ratio'] == pytest.approx(1.0)


def test_period_stats_detects_stall_regression_and_duplicate():
    stamps = [0.0, 0.1, 0.2, 0.7, 0.8, 0.8, 0.75, 0.9]
    stats = period_stats(stamps, expected_rate_hz=10.0)
    assert stats['stalls'] == 1             # 0.2 -> 0.7 is 5 periods
    assert stats['duplicate_stamps'] == 1   # 0.8, 0.8
    assert stats['timestamp_regressions'] == 1  # 0.8 -> 0.75
    assert stats['period_max_s'] == pytest.approx(0.5)


def test_period_stats_short_streams():
    assert period_stats([]) == {'count': 0}
    assert period_stats([1.0]) == {'count': 1}


def test_trajectory_length_of_square():
    square = Trajectory.from_rows([(0, 0, 0, 0), (1, 1, 0, 0), (2, 1, 1, 0), (3, 0, 1, 0),
                                   (4, 0, 0, 0)])
    assert trajectory_length(square) == pytest.approx(4.0)


def test_interpolation_and_yaw_wrap():
    ref = Trajectory.from_rows([(0.0, 0.0, 0.0, math.pi - 0.1), (1.0, 2.0, 4.0, -math.pi + 0.1)])
    out, valid = interpolate(ref, [0.5, 0.0, 1.0, -0.1, 1.1], max_gap_s=2.0)
    assert valid.tolist() == [True, True, True, False, False]
    assert out.x[0] == pytest.approx(1.0)
    assert out.y[0] == pytest.approx(2.0)
    # Halfway along the 0.2 rad arc through +/-pi, not through 0.
    assert abs(out.yaw[0]) == pytest.approx(math.pi)


def test_interpolation_refuses_to_bridge_dropouts():
    ref = Trajectory.from_rows([(0.0, 0, 0, 0), (0.05, 1, 0, 0), (1.0, 2, 0, 0)])
    _, valid = interpolate(ref, [0.02, 0.5, 1.0], max_gap_s=0.1)
    assert valid.tolist() == [True, False, True]


def test_associate_handles_different_rates():
    gt = line(n=1001, dt=0.01)       # 100 Hz
    est = line(n=101, dt=0.1)        # 10 Hz, same motion
    e, g = associate(est, gt)
    assert len(e) == len(g) == 101
    np.testing.assert_allclose(e.x, g.x, atol=1e-9)


def test_umeyama_recovers_rigid_transform():
    rng = np.random.default_rng(3)
    src = rng.uniform(-10, 10, size=(50, 2))
    theta = 0.7
    r_true = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    dst = src @ r_true.T + [3.0, -2.0]
    r, t = umeyama(src, dst)
    np.testing.assert_allclose(r, r_true, atol=1e-12)
    np.testing.assert_allclose(t, [3.0, -2.0], atol=1e-12)
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_umeyama_never_returns_reflection():
    src = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    dst = src * [1.0, -1.0]     # mirror image: best proper rotation, not a reflection
    r, _ = umeyama(src, dst)
    assert np.linalg.det(r) == pytest.approx(1.0)


def test_ate_is_zero_for_rigidly_moved_copy():
    gt = line()
    est = transform(gt, 1.2, -5.0, 7.0)
    ate = absolute_trajectory_error(est, gt)
    assert ate['translation_m']['rmse'] == pytest.approx(0.0, abs=1e-9)
    assert ate['rotation_deg']['max'] == pytest.approx(0.0, abs=1e-9)


def test_ate_of_symmetric_lateral_noise():
    # Alternating +/-d offsets on a straight line: by symmetry the optimal
    # alignment is the identity, so every error is exactly d.
    gt = line()
    d = 0.1
    est = Trajectory(gt.t, gt.x, gt.y + d * (-1.0) ** np.arange(len(gt)), gt.yaw)
    ate = absolute_trajectory_error(est, gt)
    assert ate['translation_m']['rmse'] == pytest.approx(d, rel=1e-3)
    assert ate['translation_m']['max'] == pytest.approx(d, rel=1e-2)


def test_drift_of_heading_error_at_start():
    # Estimate rotated by theta about the start: after first-pose alignment the
    # endpoint error is the chord 2 L sin(theta / 2).
    gt = line(length=10.0)
    theta = 0.05
    est = transform(gt, 0.0, 0.0, 0.0)
    est = Trajectory(est.t, gt.x * math.cos(theta), gt.x * math.sin(theta), gt.yaw)
    result = drift(est, gt)
    assert result['final_translation_m'] == pytest.approx(2 * 10.0 * math.sin(theta / 2), rel=1e-6)
    assert result['final_translation_percent_of_length'] == pytest.approx(
        100 * 2 * math.sin(theta / 2), rel=1e-6)


def test_rpe_zero_for_rigidly_moved_copy_and_scale_error_detected():
    gt = line(n=201, length=20.0)
    assert relative_pose_error(transform(gt, 0.4, 1, 2), gt, 1.0)['translation_m']['max'] == \
        pytest.approx(0.0, abs=1e-9)
    # 2% scale error: every 1 m segment is 0.02 m too long.
    scaled = Trajectory(gt.t, gt.x * 1.02, gt.y, gt.yaw)
    rpe = relative_pose_error(scaled, gt, 1.0)
    assert rpe['translation_m']['mean'] == pytest.approx(0.02, rel=1e-6)
    assert rpe['pairs'] == 191


def test_summarise():
    assert summarise([]) == {}
    s = summarise([1.0, 2.0, 3.0, 4.0])
    assert s['mean'] == pytest.approx(2.5)
    assert s['max'] == 4.0
    assert s['count'] == 4

# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Evaluation mathematics, independent of ROS.

Trajectories are planar (SE(2)) because the robot and RTAB-Map are configured
for 3-DoF motion: a trajectory is a `Trajectory` of timestamps [s], x, y [m]
and yaw [rad]. See docs/benchmarking.md for the definitions and their
limitations.
"""

from dataclasses import dataclass

import numpy as np


@dataclass
class Trajectory:
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    yaw: np.ndarray

    @staticmethod
    def from_rows(rows):
        """Build from an iterable of (t, x, y, yaw) rows, sorted by time."""
        a = np.asarray(rows, dtype=float).reshape(-1, 4)
        a = a[np.argsort(a[:, 0], kind='stable')]
        return Trajectory(a[:, 0], a[:, 1], a[:, 2], a[:, 3])

    def __len__(self):
        return len(self.t)

    @property
    def xy(self):
        return np.column_stack([self.x, self.y])

    def subset(self, mask):
        return Trajectory(self.t[mask], self.x[mask], self.y[mask], self.yaw[mask])


def wrap_angle(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def period_stats(stamps, expected_rate_hz=None, stall_factor=2.0):
    """
    Timing statistics of a message stream from its header stamps [s].

    Stamps are taken in arrival order. A non-positive step is a timestamp
    regression (zero: duplicate). A stall is a step longer than stall_factor
    expected periods. The observed rate is (n - 1) / (t_last - t_first).
    """
    t = np.asarray(stamps, dtype=float)
    result = {'count': int(len(t))}
    if len(t) < 2:
        return result
    dt = np.diff(t)
    forward = dt[dt > 0.0]
    span = t.max() - t.min()
    result.update({
        'rate_hz': (len(t) - 1) / span if span > 0 else float('nan'),
        'period_mean_s': float(forward.mean()) if len(forward) else float('nan'),
        'period_min_s': float(forward.min()) if len(forward) else float('nan'),
        'period_max_s': float(forward.max()) if len(forward) else float('nan'),
        'period_std_s': float(forward.std()) if len(forward) else float('nan'),
        'timestamp_regressions': int(np.count_nonzero(dt < 0.0)),
        'duplicate_stamps': int(np.count_nonzero(dt == 0.0)),
    })
    if expected_rate_hz:
        expected_period = 1.0 / expected_rate_hz
        result['expected_rate_hz'] = float(expected_rate_hz)
        result['rate_ratio'] = result['rate_hz'] / expected_rate_hz
        result['stalls'] = int(np.count_nonzero(forward > stall_factor * expected_period))
    return result


def interpolate(ref: Trajectory, t_query, max_gap_s=0.1):
    """
    Sample `ref` at `t_query` by linear interpolation (yaw along the shortest arc).

    Returns (trajectory, mask): the samples, and which queries were valid. A query
    is invalid outside the reference time span or where the two bracketing
    reference samples are more than max_gap_s apart, so that a dropout is never
    bridged by a long straight line.
    """
    tq = np.asarray(t_query, dtype=float)
    idx = np.searchsorted(ref.t, tq)
    valid = (idx > 0) & (idx < len(ref.t))
    exact = np.isin(tq, ref.t)
    lo = np.clip(idx - 1, 0, len(ref.t) - 1)
    hi = np.clip(idx, 0, len(ref.t) - 1)
    valid &= (ref.t[hi] - ref.t[lo]) <= max_gap_s
    valid |= exact
    span = ref.t[hi] - ref.t[lo]
    w = np.divide(tq - ref.t[lo], span, out=np.zeros_like(tq), where=span > 0)
    x = ref.x[lo] + w * (ref.x[hi] - ref.x[lo])
    y = ref.y[lo] + w * (ref.y[hi] - ref.y[lo])
    yaw = wrap_angle(ref.yaw[lo] + w * wrap_angle(ref.yaw[hi] - ref.yaw[lo]))
    return Trajectory(tq[valid], x[valid], y[valid], yaw[valid]), valid


def associate(estimate: Trajectory, ground_truth: Trajectory, max_gap_s=0.1):
    """Pair each estimate sample with ground truth interpolated at the same stamp."""
    gt, valid = interpolate(ground_truth, estimate.t, max_gap_s)
    return estimate.subset(valid), gt


def umeyama(source, target):
    """
    Rigid transform (R, t) minimising sum ||target_i - (R source_i + t)||^2.

    Umeyama (1991) without scale; the reflection guard keeps det(R) = +1.
    Works in any dimension; inputs are (N, d) arrays of corresponding points.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    mu_s = source.mean(axis=0)
    mu_t = target.mean(axis=0)
    cov = (target - mu_t).T @ (source - mu_s) / len(source)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(source.shape[1])
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[-1, -1] = -1.0
    r = u @ s @ vt
    return r, mu_t - r @ mu_s


def apply_se2(traj: Trajectory, r, t):
    xy = traj.xy @ r.T + t
    dyaw = np.arctan2(r[1, 0], r[0, 0])
    return Trajectory(traj.t, xy[:, 0], xy[:, 1], wrap_angle(traj.yaw + dyaw))


def align_first_pose(estimate: Trajectory, ground_truth: Trajectory):
    """Express the estimate in the ground-truth frame by matching the first pose exactly."""
    dyaw = ground_truth.yaw[0] - estimate.yaw[0]
    r = np.array([[np.cos(dyaw), -np.sin(dyaw)], [np.sin(dyaw), np.cos(dyaw)]])
    t = ground_truth.xy[0] - r @ estimate.xy[0]
    return apply_se2(estimate, r, t)


def trajectory_length(traj: Trajectory):
    return float(np.sum(np.linalg.norm(np.diff(traj.xy, axis=0), axis=1)))


def error_summary(errors):
    e = np.asarray(errors, dtype=float)
    return {
        'rmse': float(np.sqrt(np.mean(e ** 2))),
        'mean': float(e.mean()),
        'median': float(np.median(e)),
        'max': float(e.max()),
        'std': float(e.std()),
    }


def absolute_trajectory_error(estimate: Trajectory, ground_truth: Trajectory):
    """
    ATE (Sturm et al., 2012): translation error after the least-squares rigid
    alignment of the associated positions. `estimate` and `ground_truth` must
    already be associated sample by sample.
    """
    r, t = umeyama(estimate.xy, ground_truth.xy)
    aligned = apply_se2(estimate, r, t)
    err = np.linalg.norm(aligned.xy - ground_truth.xy, axis=1)
    yaw_err = np.abs(wrap_angle(aligned.yaw - ground_truth.yaw))
    return {
        'translation_m': error_summary(err),
        'rotation_deg': error_summary(np.degrees(yaw_err)),
        'samples': int(len(err)),
    }


def drift(estimate: Trajectory, ground_truth: Trajectory):
    """
    Error growth as seen online: align only the first pose, then report the
    final and maximum position error and the final heading error.
    """
    aligned = align_first_pose(estimate, ground_truth)
    err = np.linalg.norm(aligned.xy - ground_truth.xy, axis=1)
    length = trajectory_length(ground_truth)
    return {
        'final_translation_m': float(err[-1]),
        'max_translation_m': float(err.max()),
        'final_translation_percent_of_length': float(100.0 * err[-1] / length) if length else 0.0,
        'final_yaw_deg': float(np.degrees(abs(wrap_angle(aligned.yaw[-1] - ground_truth.yaw[-1])))),
    }


def _se2(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def relative_pose_error(estimate: Trajectory, ground_truth: Trajectory, delta_m=1.0):
    """
    RPE over a travelled distance delta_m (KITTI-style segments).

    For each i, j is the first index with path length s_j - s_i >= delta_m
    along the ground truth, and the error is
        E = (Q_i^-1 Q_j)^-1 (P_i^-1 P_j)
    with Q ground truth and P estimate. Independent of any global alignment.
    """
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(ground_truth.xy, axis=0), axis=1))])
    # 1 nm tolerance: cumulative sums of equal steps can land a hair short of delta_m.
    j_of_i = np.searchsorted(s, s + delta_m - 1e-9)
    pairs = [(i, j) for i, j in enumerate(j_of_i) if j < len(s)]
    if not pairs:
        return {'delta_m': delta_m, 'pairs': 0}
    trans, rot = [], []
    for i, j in pairs:
        q = np.linalg.inv(_se2(ground_truth.x[i], ground_truth.y[i], ground_truth.yaw[i])) @ \
            _se2(ground_truth.x[j], ground_truth.y[j], ground_truth.yaw[j])
        p = np.linalg.inv(_se2(estimate.x[i], estimate.y[i], estimate.yaw[i])) @ \
            _se2(estimate.x[j], estimate.y[j], estimate.yaw[j])
        e = np.linalg.inv(q) @ p
        trans.append(np.hypot(e[0, 2], e[1, 2]))
        rot.append(abs(np.arctan2(e[1, 0], e[0, 0])))
    return {
        'delta_m': delta_m,
        'pairs': len(pairs),
        'translation_m': error_summary(trans),
        'rotation_deg': error_summary(np.degrees(rot)),
    }


def evaluate_trajectory(estimate: Trajectory, ground_truth: Trajectory, max_gap_s=0.1,
                        rpe_delta_m=1.0):
    """Associate, then compute ATE, drift and RPE. Returns None if nothing associates."""
    est, gt = associate(estimate, ground_truth, max_gap_s)
    if len(est) < 3:
        return None
    return {
        'samples': int(len(est)),
        'duration_s': float(est.t[-1] - est.t[0]),
        'length_m': trajectory_length(gt),
        'ate': absolute_trajectory_error(est, gt),
        'drift': drift(est, gt),
        'rpe': relative_pose_error(est, gt, rpe_delta_m),
    }


def summarise(values):
    """mean/p50/p95/max of a sample, or an empty dict."""
    v = np.asarray(values, dtype=float)
    if v.size == 0:
        return {}
    return {
        'mean': float(v.mean()),
        'p50': float(np.percentile(v, 50)),
        'p95': float(np.percentile(v, 95)),
        'max': float(v.max()),
        'count': int(v.size),
    }

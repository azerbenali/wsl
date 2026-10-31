# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
import glob
import math
import os

import numpy as np
import pytest

from wsl_benchmark.route import densify, pursuit_command, PursuitConfig, PursuitState, Scenario

SCENARIO_DIR = os.path.join(os.path.dirname(__file__), '..', 'config', 'scenarios')
SCENARIOS = glob.glob(os.path.join(SCENARIO_DIR, '*.yaml'))


@pytest.mark.parametrize('path', SCENARIOS, ids=os.path.basename)
def test_scenarios_are_consistent(path):
    scenario = Scenario.load(path)
    if scenario.waypoints:
        start = scenario.waypoints[0]
        assert start == pytest.approx((scenario.spawn['x'], scenario.spawn['y'])), \
            'route must start at the spawn pose'


def test_scenario_rejects_unknown_robot_argument(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('spawn: {x: 0, y: 0, yaw: 0}\nrobot: {lidar_chanels: 8}\n')
    with pytest.raises(ValueError, match='lidar_chanels'):
        Scenario.load(str(path))


def test_densify_spacing_and_endpoints():
    path = densify([(0, 0), (1, 0), (1, 2)], spacing=0.1)
    np.testing.assert_allclose(path[0], (0, 0))
    np.testing.assert_allclose(path[-1], (1, 2))
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    assert steps.max() <= 0.1 + 1e-9


def simulate(path, x, y, yaw, cfg, dt=0.05, max_steps=20000):
    state = PursuitState()
    trace = []
    for _ in range(max_steps):
        v, w = pursuit_command(x, y, yaw, path, state, cfg)
        if state.done:
            return trace, True
        yaw += w * dt
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        trace.append((x, y))
    return trace, False


def test_pursuit_follows_a_closed_loop_to_the_end():
    # A loop whose end coincides with its start must not finish early.
    path = densify([(0, 0), (10, 0), (10, 6), (0, 6), (0, 0)])
    cfg = PursuitConfig(speed_mps=0.6)
    trace, done = simulate(path, 0.0, 0.0, 0.0, cfg)
    assert done
    travelled = np.sum(np.linalg.norm(np.diff(np.array(trace), axis=0), axis=1))
    assert travelled > 30.0
    # Stays within a 3 m aisle (1.5 m of the centre line) throughout.
    deviation = [np.min(np.linalg.norm(path - p, axis=1)) for p in trace]
    assert max(deviation) < 0.6


def test_pursuit_turns_in_place_when_facing_away():
    path = densify([(0, 0), (-5, 0)])
    v, w = pursuit_command(0.0, 0.0, 0.0, path, PursuitState(), PursuitConfig())
    assert v == 0.0
    assert abs(w) == PursuitConfig().max_angular_rps

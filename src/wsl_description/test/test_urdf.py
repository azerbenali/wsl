# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Structural and physical checks on the expanded WSL AMR model."""

import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
from urdf_parser_py.urdf import Sphere, URDF

URDF_DIR = os.environ.get(
    'WSL_DESCRIPTION_URDF_DIR',
    os.path.join(os.path.dirname(__file__), '..', 'urdf'))
MODEL = os.path.join(URDF_DIR, 'wsl_amr.urdf.xacro')

REQUIRED_LINKS = {
    'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link',
    'lidar_link', 'camera_link', 'camera_optical_frame', 'imu_link',
}
REQUIRED_JOINTS = {'left_wheel_joint', 'right_wheel_joint'}


def expand(**args):
    mappings = {k: str(v) for k, v in args.items()}
    return xacro.process_file(MODEL, mappings=mappings).toprettyxml(indent='  ')


@pytest.fixture(scope='module')
def urdf_xml():
    return expand()


@pytest.fixture(scope='module')
def robot(urdf_xml):
    return URDF.from_xml_string(urdf_xml)


def rpy_to_matrix(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def test_tree_structure(robot):
    assert robot.get_root() == 'base_footprint'
    assert REQUIRED_LINKS <= {link.name for link in robot.links}
    assert REQUIRED_JOINTS <= {joint.name for joint in robot.joints}
    for joint in robot.joints:
        if joint.name.endswith('wheel_joint'):
            assert joint.type == 'continuous'
            assert list(joint.axis) == [0, 1, 0]
        else:
            assert joint.type == 'fixed', f'{joint.name} should be fixed'


def test_inertia_is_physical(robot):
    """Each inertia tensor must be positive definite and obey the triangle inequality."""
    for link in robot.links:
        if link.inertial is None:
            continue
        assert link.inertial.mass > 0.0, link.name
        i = link.inertial.inertia
        tensor = np.array([[i.ixx, i.ixy, i.ixz], [i.ixy, i.iyy, i.iyz], [i.ixz, i.iyz, i.izz]])
        eig = np.linalg.eigvalsh(tensor)
        assert np.all(eig > 0.0), f'{link.name}: inertia not positive definite {eig}'
        a, b, c = eig
        assert a + b >= c and a + c >= b and b + c >= a, \
            f'{link.name}: violates triangle inequality'


def test_total_mass_is_plausible(robot):
    total = sum(link.inertial.mass for link in robot.links if link.inertial)
    assert 25.0 < total < 40.0


def test_wheels_touch_ground(robot):
    base_joint = robot.joint_map['base_joint']
    axle_height = base_joint.origin.xyz[2]
    for side in ('left', 'right'):
        wheel = robot.link_map[f'{side}_wheel_link']
        radius = wheel.collisions[0].geometry.radius
        assert radius == pytest.approx(axle_height)
        assert robot.joint_map[f'{side}_wheel_joint'].origin.xyz[2] == pytest.approx(0.0)


def test_casters_clear_ground(robot):
    axle_height = robot.joint_map['base_joint'].origin.xyz[2]
    casters = [c for c in robot.link_map['base_link'].collisions
               if isinstance(c.geometry, Sphere)]
    assert len(casters) == 2
    for caster in casters:
        bottom = axle_height + caster.origin.xyz[2] - caster.geometry.radius
        assert 0.0 < bottom <= 0.002, f'caster at x={caster.origin.xyz[0]} bottom at {bottom} m'


def test_camera_optical_frame_convention(robot):
    """Optical z must point along base x, optical x along base -y, optical y along base -z."""
    rot = rpy_to_matrix(robot.joint_map['camera_optical_joint'].origin.rpy)
    np.testing.assert_allclose(rot @ [0, 0, 1], [1, 0, 0], atol=1e-9)
    np.testing.assert_allclose(rot @ [1, 0, 0], [0, -1, 0], atol=1e-9)
    np.testing.assert_allclose(rot @ [0, 1, 0], [0, 0, -1], atol=1e-9)


def test_lidar_height(robot):
    axle_height = robot.joint_map['base_joint'].origin.xyz[2]
    lidar_z = robot.joint_map['lidar_joint'].origin.xyz[2]
    assert axle_height + lidar_z == pytest.approx(0.50)


def find_plugin(root, name):
    return [p for p in root.iter('plugin') if p.get('name') == name]


def test_sensor_frames_match_links(urdf_xml):
    root = ET.fromstring(urdf_xml)
    links = {link.get('name') for link in root.iter('link')}
    sensors = list(root.iter('sensor'))
    assert {s.get('name') for s in sensors} == {'lidar', 'camera', 'imu'}
    for sensor in sensors:
        assert sensor.find('gz_frame_id').text in links


def test_lidar_horizontal_scan_does_not_overlap(urdf_xml):
    horizontal = ET.fromstring(urdf_xml).find('.//sensor[@name="lidar"]//horizontal')
    samples = int(horizontal.find('samples').text)
    span = float(horizontal.find('max_angle').text) - float(horizontal.find('min_angle').text)
    assert span == pytest.approx(2 * math.pi * (samples - 1) / samples)


def test_odometry_calibration_scales(robot):
    root = ET.fromstring(expand(odom_wheel_radius_scale=0.98, odom_wheel_separation_scale=1.05))
    drive = find_plugin(root, 'gz::sim::systems::DiffDrive')[0]
    assert float(drive.find('wheel_radius').text) == pytest.approx(0.08 * 0.98)
    assert float(drive.find('wheel_separation').text) == pytest.approx(0.44 * 1.05)


def test_ground_truth_can_be_disabled():
    name = 'gz::sim::systems::OdometryPublisher'
    assert len(find_plugin(ET.fromstring(expand()), name)) == 1
    assert len(find_plugin(ET.fromstring(expand(ground_truth='false')), name)) == 0


@pytest.mark.skipif(shutil.which('check_urdf') is None, reason='check_urdf not installed')
def test_check_urdf(urdf_xml, tmp_path):
    path = tmp_path / 'wsl_amr.urdf'
    path.write_text(urdf_xml)
    result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr

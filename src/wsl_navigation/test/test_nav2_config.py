# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""
Consistency checks between the Nav2 configuration, the robot model and the rest of the system.

Robot quantities (chassis box, drive limits, camera and wheel positions) are read
from the expanded xacro, frames from docs/frames.md and topic names from the
bridge configuration and docs/architecture.md, so none of them is written twice.
No simulator is needed.
"""

import importlib.util
import math
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

NAV_DIR = os.environ.get('WSL_NAVIGATION_DIR', os.path.join(os.path.dirname(__file__), '..'))
SRC_DIR = os.path.dirname(os.path.abspath(NAV_DIR))
DOCS_DIR = os.path.join(os.path.dirname(SRC_DIR), 'docs')
CONFIG = os.path.join(NAV_DIR, 'config', 'nav2.yaml')
LAUNCH = os.path.join(NAV_DIR, 'launch', 'navigation.launch.py')
# wsl_sim depends on this package, so its bridge file is read from the source tree.
BRIDGE = os.path.join(SRC_DIR, 'wsl_sim', 'config', 'bridge.yaml')

# Topic each node publishes its velocity on before remapping (Nav2 Jazzy).
VELOCITY_OUTPUTS = {'controller_server': 'cmd_vel', 'behavior_server': 'cmd_vel',
                    'velocity_smoother': 'cmd_vel_smoothed'}
VELOCITY_SMOOTHER_INPUT = 'cmd_vel'


@pytest.fixture(scope='module')
def config():
    with open(CONFIG) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def launch():
    spec = importlib.util.spec_from_file_location('navigation_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def robot():
    urdf = os.path.join(
        get_package_share_directory('wsl_description'), 'urdf', 'wsl_amr.urdf.xacro')
    return ET.fromstring(xacro.process_file(urdf).toxml())


def params(config, node):
    """ros__parameters of a node; costmaps are nested under their server's name."""
    if node.endswith('_costmap'):
        return config[node][node]['ros__parameters']
    return config[node]['ros__parameters']


def all_params(config):
    for name, body in config.items():
        if 'ros__parameters' in body:
            yield name, body['ros__parameters']
        else:
            for sub, nested in body.items():
                yield f'{name}/{sub}', nested['ros__parameters']


def chassis_box(robot):
    """The single box collision of base_link: the chassis mesh bounding box."""
    base = robot.find('link[@name="base_link"]')
    (collision,) = [c for c in base.findall('collision') if c.find('geometry/box') is not None]
    size = [float(v) for v in collision.find('geometry/box').get('size').split()]
    centre = [float(v) for v in collision.find('origin').get('xyz').split()]
    return size, centre


def drive_limits(robot):
    (drive,) = [p for p in robot.iter('plugin')
                if p.get('name') == 'gz::sim::systems::MecanumDrive']
    return {tag: float(drive.find(tag).text) for tag in (
        'max_velocity', 'min_velocity', 'max_acceleration', 'min_acceleration')}


def joint_xyz(robot, name):
    return [float(v) for v in robot.find(f'joint[@name="{name}"]/origin').get('xyz').split()]


def footprint(config, costmap):
    return yaml.safe_load(params(config, costmap)['footprint'])


def doc_frame_parents():
    """Parent of each frame from the frame table in docs/frames.md."""
    parents = {}
    publishers = {}
    with open(os.path.join(DOCS_DIR, 'frames.md')) as f:
        for line in f:
            row = re.match(r'\| `([a-z_]+)` \| (`([a-z_]+)`|–) \| ([^|]*) \|', line)
            if row:
                parents[row.group(1)] = row.group(3)
                publishers[row.group(1)] = row.group(4)
    return parents, publishers


def test_yaml_parses_and_every_node_uses_sim_time(config):
    nodes = dict(all_params(config))
    assert nodes
    for name, values in nodes.items():
        assert values['use_sim_time'] is True, name


def test_launched_nodes_and_configured_nodes_match(config, launch):
    """No orphan configuration and no server running on defaults."""
    launched = {executable for _, executable, _ in launch.NODES}
    assert set(launch.LIFECYCLE_NODES) == launched
    assert len(launch.LIFECYCLE_NODES) == len(launched)
    configured = {name.split('/')[0] for name, _ in all_params(config)}
    assert configured == launched | {'local_costmap', 'global_costmap'}


def test_no_amcl_and_no_map_server(config, launch):
    """RTAB-Map is the only source of map -> odom and of the map."""
    for forbidden in ('amcl', 'map_server', 'nav2_amcl', 'nav2_map_server'):
        assert forbidden not in config
        assert all(forbidden not in (package, executable)
                   for package, executable, _ in launch.NODES)
        assert forbidden not in launch.LIFECYCLE_NODES
    for name, values in all_params(config):
        assert 'yaml_filename' not in values, name


def test_frames_match_docs(config):
    parents, publishers = doc_frame_parents()
    # map is the root and RTAB-Map publishes map -> odom; odometry owns odom -> base_footprint.
    assert parents['map'] is None
    assert parents['odom'] == 'map' and 'rtabmap' in publishers['odom']
    assert parents['base_footprint'] == 'odom' and 'MecanumDrive' in publishers['base_footprint']

    for node in ('bt_navigator', 'global_costmap'):
        assert params(config, node)['global_frame'] == 'map', node
    assert params(config, 'local_costmap')['global_frame'] == 'odom'
    behavior = params(config, 'behavior_server')
    assert (behavior['global_frame'], behavior['local_frame']) == ('map', 'odom')
    for node in ('bt_navigator', 'global_costmap', 'local_costmap', 'behavior_server'):
        assert params(config, node)['robot_base_frame'] == 'base_footprint', node


def test_odometry_topic(config):
    with open(BRIDGE) as f:
        odom = [e for e in yaml.safe_load(f) if e['ros_type_name'] == 'nav_msgs/msg/Odometry']
    assert [e['ros_topic_name'] for e in odom] == ['/odom']
    for node in ('bt_navigator', 'controller_server', 'velocity_smoother'):
        assert params(config, node)['odom_topic'] == '/odom', node


def test_footprint_is_the_chassis_box(config, robot):
    (length, width, _), (x, y, _) = chassis_box(robot)
    assert (x, y) == (0.0, 0.0), 'chassis box must be centred on base_footprint'
    expected = sorted((sx * length / 2, sy * width / 2) for sx in (1, -1) for sy in (1, -1))
    for costmap in ('local_costmap', 'global_costmap'):
        values = params(config, costmap)
        assert 'robot_radius' not in values, costmap
        points = sorted(tuple(p) for p in footprint(config, costmap))
        assert points == pytest.approx(expected, abs=1e-6), costmap
        assert 0.0 < values['footprint_padding'] <= 0.1, costmap


def test_padded_footprint_covers_wheels_and_camera(config, robot):
    """The wheel collision spheres stick out at the sides, the camera at the front."""
    (length, width, _), _ = chassis_box(robot)
    padding = params(config, 'local_costmap')['footprint_padding']
    for wheel in ('front_left', 'front_right', 'back_left', 'back_right'):
        x, y, _ = joint_xyz(robot, f'{wheel}_wheel_joint')
        link = robot.find(f'link[@name="{wheel}_wheel_link"]')
        radius = float(link.find('collision/geometry/sphere').get('radius'))
        assert abs(x) + radius <= length / 2 + padding, wheel
        assert abs(y) + radius <= width / 2 + padding, wheel
    camera_x = joint_xyz(robot, 'camera_joint')[0]
    camera_depth = float(robot.find('link[@name="camera_link"]/visual/geometry/box')
                         .get('size').split()[0])
    assert camera_x + camera_depth / 2 <= length / 2 + padding


def test_inflation_reaches_the_circumscribed_radius(config):
    """MPPI's footprint check skips points whose cost proves they are clear; that needs this."""
    for costmap in ('local_costmap', 'global_costmap'):
        values = params(config, costmap)
        padding = values['footprint_padding']
        circumscribed = max(math.hypot(abs(px) + padding, abs(py) + padding)
                            for px, py in footprint(config, costmap))
        assert values['inflation_layer']['inflation_radius'] >= circumscribed, costmap


def test_velocity_limits_are_inside_the_drive_limits(config, robot):
    limits = drive_limits(robot)
    v_max = min(limits['max_velocity'], -limits['min_velocity'])
    a_max = min(limits['max_acceleration'], -limits['min_acceleration'])

    mppi = params(config, 'controller_server')['FollowPath']
    for name in ('vx_max', 'vy_max', 'wz_max'):
        assert 0.0 < mppi[name] <= v_max, name
    assert -v_max <= mppi['vx_min'] < 0.0
    assert math.hypot(mppi['vx_max'], mppi['vy_max']) <= v_max
    for name in ('ax_max', 'ay_max', 'az_max'):
        assert 0.0 < mppi[name] <= a_max, name
    for name in ('ax_min', 'ay_min'):
        assert -a_max <= mppi[name] < 0.0, name

    smoother = params(config, 'velocity_smoother')
    for upper, lower in (('max_velocity', 'min_velocity'), ('max_accel', 'max_decel')):
        bound = v_max if upper == 'max_velocity' else a_max
        for axis in range(3):
            assert 0.0 < smoother[upper][axis] <= bound, (upper, axis)
            assert -bound <= smoother[lower][axis] < 0.0, (lower, axis)
    assert math.hypot(smoother['max_velocity'][0], smoother['max_velocity'][1]) <= v_max

    behavior = params(config, 'behavior_server')
    assert 0.0 < behavior['min_rotational_vel'] <= behavior['max_rotational_vel'] <= v_max
    assert 0.0 < behavior['rotational_acc_lim'] <= a_max


def test_mppi_plans_only_what_the_smoother_passes(config):
    """Otherwise the smoother would clip the optimal trajectory MPPI is tracking."""
    mppi = params(config, 'controller_server')['FollowPath']
    smoother = params(config, 'velocity_smoother')
    assert [mppi['vx_max'], mppi['vy_max'], mppi['wz_max']] == smoother['max_velocity']
    assert [mppi['vx_min'], -mppi['vy_max'], -mppi['wz_max']] == smoother['min_velocity']
    assert [mppi['ax_max'], mppi['ay_max'], mppi['az_max']] == smoother['max_accel']
    assert [mppi['ax_min'], mppi['ay_min'], -mppi['az_max']] == smoother['max_decel']


def test_mppi_is_omnidirectional(config):
    controller = params(config, 'controller_server')
    assert controller['controller_plugins'] == ['FollowPath']
    mppi = controller['FollowPath']
    assert mppi['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert mppi['motion_model'] == 'Omni'
    assert mppi['vy_max'] > 0.0 and mppi['vy_std'] > 0.0
    # A diff-drive threshold here would zero the lateral velocity fed back to MPPI.
    assert controller['min_y_velocity_threshold'] <= 0.01
    assert mppi['model_dt'] * controller['controller_frequency'] == pytest.approx(1.0)
    assert mppi['prune_distance'] >= mppi['vx_max'] * mppi['time_steps'] * mppi['model_dt']
    for critic in mppi['critics']:
        assert mppi[critic]['enabled'] is True, critic
    assert 'TwirlingCritic' in mppi['critics']
    assert mppi['CostCritic']['consider_footprint'] is True


def test_planner_is_smac_2d_through_unknown_space(config):
    planner = params(config, 'planner_server')
    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert planner['GridBased']['allow_unknown'] is True
    assert params(config, 'global_costmap')['track_unknown_space'] is True


def test_costmap_topics_are_the_real_topics(config):
    with open(BRIDGE) as f:
        bridge = {e['ros_topic_name']: e for e in yaml.safe_load(f)}
    with open(os.path.join(DOCS_DIR, 'architecture.md')) as f:
        interfaces = dict(re.findall(r'^\| `(/[^`]+)` \| `([^`]+)`', f.read(), re.MULTILINE))

    static = params(config, 'global_costmap')['static_layer']
    assert params(config, 'global_costmap')['plugins'] == ['static_layer', 'inflation_layer']
    assert static['plugin'] == 'nav2_costmap_2d::StaticLayer'
    assert interfaces[static['map_topic']] == 'nav_msgs/OccupancyGrid'
    assert static['map_topic'] == '/rtabmap/map'
    # RTAB-Map publishes the map transient local; a volatile subscriber that starts
    # after the last update would wait for the next one with an empty costmap.
    assert static['map_subscribe_transient_local'] is True

    local = params(config, 'local_costmap')
    assert local['plugins'] == ['voxel_layer', 'inflation_layer']
    assert local['rolling_window'] is True
    voxel = local['voxel_layer']
    assert voxel['plugin'] == 'nav2_costmap_2d::VoxelLayer'
    (source,) = voxel['observation_sources'].split()
    lidar = voxel[source]
    assert lidar['topic'] == '/lidar/points'
    assert bridge[lidar['topic']]['ros_type_name'] == 'sensor_msgs/msg/PointCloud2'
    assert lidar['data_type'] == 'PointCloud2'


def test_voxel_heights_drop_the_floor_and_keep_the_robot_height(config, robot):
    voxel = params(config, 'local_costmap')['voxel_layer']
    lidar = voxel[voxel['observation_sources']]
    # The 1 cm range noise moves floor returns by millimetres; 10 cm leaves a margin.
    assert lidar['min_obstacle_height'] >= 0.05
    # Everything up to the top of the robot (the LiDAR) must be an obstacle.
    lidar_top = joint_xyz(robot, 'base_joint')[2] + joint_xyz(robot, 'lidar_joint')[2] + float(
        robot.find('link[@name="lidar_link"]/collision/geometry/cylinder').get('length')) / 2
    assert lidar['max_obstacle_height'] > lidar_top
    assert voxel['max_obstacle_height'] >= lidar['max_obstacle_height']
    # The voxel grid must span the marked band, or marks above its top are lost.
    top = voxel['origin_z'] + voxel['z_voxels'] * voxel['z_resolution']
    assert top >= lidar['max_obstacle_height'] - 1e-9
    assert voxel['z_voxels'] <= 16


def test_cmd_vel_chain_ends_in_unstamped_twist(config, launch):
    remap = {executable: dict(remappings) for _, executable, remappings in launch.NODES}

    def resolve(node, topic):
        return remap[node].get(topic, topic)

    smoother_in = resolve('velocity_smoother', VELOCITY_SMOOTHER_INPUT)
    for node in ('controller_server', 'behavior_server'):
        assert resolve(node, VELOCITY_OUTPUTS[node]) == smoother_in, node
    assert smoother_in != 'cmd_vel'
    # Only the smoother reaches the robot.
    final = {node: resolve(node, topic) for node, topic in VELOCITY_OUTPUTS.items()}
    assert [node for node, topic in final.items() if topic == 'cmd_vel'] == ['velocity_smoother']

    for node in VELOCITY_OUTPUTS:
        assert params(config, node)['enable_stamped_cmd_vel'] is False, node

    with open(BRIDGE) as f:
        (entry,) = [e for e in yaml.safe_load(f) if e['ros_topic_name'] == '/cmd_vel']
    assert entry['ros_type_name'] == 'geometry_msgs/msg/Twist'
    assert entry['direction'] == 'ROS_TO_GZ'


def test_behaviours_cover_the_default_tree(config):
    """navigate_to_pose_w_replanning_and_recovery.xml uses Spin, Wait and BackUp."""
    behavior = params(config, 'behavior_server')
    plugins = {behavior[name]['plugin'] for name in behavior['behavior_plugins']}
    assert {'nav2_behaviors::Spin', 'nav2_behaviors::Wait', 'nav2_behaviors::BackUp',
            'nav2_behaviors::DriveOnHeading'} <= plugins
    assert 'default_nav_to_pose_bt_xml' not in params(config, 'bt_navigator')

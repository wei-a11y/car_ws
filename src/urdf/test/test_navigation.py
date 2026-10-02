# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Isolated ROS integration, NOT a Gazebo/world alignment acceptance test."""
import math
import importlib.util
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
import yaml


SHARE = FilePath(__file__).resolve().parents[1]
DEBUG_COMMAND = '/phase8_test/cmd_vel'


def zero(command):
    return command.linear.x == 0.0 and command.angular.z == 0.0


def test_deployment_contract():
    amcl = yaml.safe_load((SHARE / 'config/amcl.yaml').read_text())['amcl']['ros__parameters']
    config = yaml.safe_load((SHARE / 'config/navigation_baseline.yaml').read_text())
    assert config['gazebo']['ros__parameters']['publish_rate'] > 20
    assert (amcl['global_frame_id'], amcl['odom_frame_id'], amcl['base_frame_id']) == (
        'map', 'odom', 'base_footprint')
    assert amcl['scan_topic'] == '/scan' and amcl['map_topic'] == '/map'
    assert amcl['tf_broadcast'] and not amcl['set_initial_pose']
    assert amcl['always_reset_initial_pose']
    assert config['planner']['ros__parameters']['planning_frame'] == 'map'
    controller = config['lqr_controller']['ros__parameters']
    assert controller['planning_frame'] == 'map'
    # Cell-centre quantization fits; the tracking bound alone must not spend the margin.
    assert math.sqrt(2) * 0.025 < controller['max_tracking_error']
    assert controller['max_tracking_error'] < config['planning_grid'][
        'ros__parameters']['safety_margin']
    rviz = yaml.safe_load((SHARE / 'config/navigation.rviz').read_text())
    assert rviz['Visualization Manager']['Global Options']['Fixed Frame'] == 'map'


def test_offline_metrics_are_per_goal_and_do_not_claim_incomplete_success():
    spec = importlib.util.spec_from_file_location(
        'phase8_metrics', SHARE.parents[1] / 'tools/phase8_metrics.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    metrics = module.Metrics()
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = 1.0
    metrics.feed('/goal_pose', goal, 1000000000)
    path = Path()
    for x in [0.0, 1.0]:
        pose = PoseStamped()
        pose.pose.position.x = x
        path.poses.append(pose)
    metrics.feed('/plan', path, 1100000000)
    metrics.feed('/plan/raw_path', path, 1100000000)
    metrics.feed('/plan/status', String(data='SUCCESS'), 1200000000)
    metrics.feed('/controller/lqr/status', String(data='TRACKING'), 1300000000)
    metrics.feed('/controller/lqr/debug', Float64MultiArray(data=[0.03, 0.2]), 1400000000)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
    transform.transform.translation.x = 0.98
    transform.transform.rotation.w = 1.0
    metrics.feed('/tf', TFMessage(transforms=[transform]), 1900000000)
    result = metrics.feed('/controller/lqr/status', String(data='GOAL_REACHED'), 2000000000)[0]
    assert result['result'] == 'GOAL_REACHED'
    assert result['navigation_time_sec'] == pytest.approx(1.0)
    assert result['planning_latency_sec'] == pytest.approx(0.2)
    assert result['processed_path_length_m'] == result['raw_path_length_m'] == 1.0
    assert result['cross_track_rmse_m'] == pytest.approx(0.03)
    assert result['heading_rmse_rad'] == pytest.approx(0.2)
    assert result['terminal_position_error_m'] == pytest.approx(0.02)
    metrics.feed('/goal_pose', goal, 3000000000)
    assert metrics.finish('INCOMPLETE', 4000000000)['result'] == 'INCOMPLETE'


class Harness:
    def __init__(self, node):
        self.node = node
        self.pose = [1.025, 1.025, 0.0]
        self.command = Twist()
        self.outputs, self.paths, self.raw_commands = [], [], []
        self.status = {}
        self.nanoseconds = 1000000000
        self.emit_scan = self.emit_clock = self.emit_tf = True
        self.obstacle = False
        self.tf = TransformBroadcaster(node)
        self.static = StaticTransformBroadcaster(node)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        sensors = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.maps = node.create_publisher(OccupancyGrid, '/map', latched)
        self.clock = node.create_publisher(Clock, '/clock', 1)
        self.odometry = node.create_publisher(Odometry, '/odom', sensors)
        self.scans = node.create_publisher(LaserScan, '/scan', sensors)
        self.initial = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.goals = node.create_publisher(PoseStamped, '/goal_pose', 1)
        node.create_subscription(Twist, DEBUG_COMMAND, self.on_command, 10)
        node.create_subscription(Twist, '/cmd_vel_raw', self.raw_commands.append, 10)
        node.create_subscription(Path, '/plan', self.paths.append, 1)
        for topic in ['/plan/status', '/controller/lqr/status', '/controller/safety/status']:
            node.create_subscription(String, topic, lambda msg, t=topic: self.on_status(t, msg),
                                     latched)
        # Synthetic sensor extrinsics matching the confirmed URDF. Not a map->odom shortcut.
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'base_footprint', 'laser_link'
        transform.transform.translation.x, transform.transform.translation.z = 0.155, 0.2825
        transform.transform.rotation.z = math.sin(3.14 / 2)
        transform.transform.rotation.w = math.cos(3.14 / 2)
        self.static.sendTransform(transform)
        self.timer = node.create_timer(0.02, self.tick)
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width = self.map.info.height = 80
        self.map.info.resolution = 0.05
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [100 if x in (0, 79) or y in (0, 79) else 0
                         for y in range(80) for x in range(80)]
        self.maps.publish(self.map)

    def on_command(self, command):
        self.command = command
        self.outputs.append(command)

    def on_status(self, topic, message):
        self.status[topic] = message.data

    def tick(self):
        if not self.emit_clock:
            return
        self.nanoseconds += 20000000
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(self.nanoseconds, 1000000000)
        self.clock.publish(clock)
        v, w = self.command.linear.x, self.command.angular.z
        self.pose[0] += v * math.cos(self.pose[2]) * 0.02
        self.pose[1] += v * math.sin(self.pose[2]) * 0.02
        self.pose[2] += w * 0.02
        stamp = clock.clock
        odom = Odometry()
        odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
        odom.header.stamp = stamp
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.pose[:2]
        odom.pose.pose.orientation.z = math.sin(self.pose[2] / 2)
        odom.pose.pose.orientation.w = math.cos(self.pose[2] / 2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        self.odometry.publish(odom)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x, transform.transform.translation.y = self.pose[:2]
        transform.transform.rotation = odom.pose.pose.orientation
        if self.emit_tf:
            self.tf.sendTransform(transform)
        if self.emit_scan and self.nanoseconds % 100000000 == 0:
            self.publish_scan(stamp)

    def publish_scan(self, stamp):
        scan = LaserScan()
        scan.header.frame_id, scan.header.stamp = 'laser_link', stamp
        scan.angle_min, scan.angle_max = 0.349, 5.934
        scan.angle_increment = (scan.angle_max - scan.angle_min) / 359
        scan.range_min, scan.range_max = 0.12, 8.0
        x = self.pose[0] + 0.155 * math.cos(self.pose[2])
        y = self.pose[1] + 0.155 * math.sin(self.pose[2])
        for i in range(360):
            angle = scan.angle_min + i * scan.angle_increment + 3.14 + self.pose[2]
            dx, dy = math.cos(angle), math.sin(angle)
            distances = []
            for value, direction in [(x, dx), (y, dy)]:
                if abs(direction) > 1e-9:
                    distances.append(((3.95 if direction > 0 else 0.05) - value) / direction)
            distance = min(distances)
            if self.obstacle and abs(math.remainder(angle - self.pose[2], 2 * math.pi)) < 0.7:
                distance = 0.4
            scan.ranges.append(float(distance))
        self.scans.publish(scan)

    def wait(self, predicate, seconds=15):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(self.node, timeout_sec=0.01)
        assert predicate(), f'Timed out: statuses={self.status}, pose={self.pose}'

    def settle(self, seconds=0.5):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def initialize(self):
        self.wait(lambda: self.initial.get_subscription_count() > 0 and
                  self.goals.get_subscription_count() > 0 and bool(self.outputs))
        self.settle(1.0)  # lifecycle activation and /map subscription
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x, pose.pose.pose.position.y = self.pose[:2]
        pose.pose.pose.orientation.w = 1.0
        # Known synthetic pose only; actual world uses RViz initialization, not these values.
        pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = 1e-8
        self.initial.publish(pose)
        self.wait(lambda: self.buffer.can_transform('map', 'base_footprint', rclpy.time.Time()))
        self.settle()

    def goal(self, x=1.325, y=1.025):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.w = 1.0
        self.goals.publish(goal)


@pytest.fixture
def navigation(tmp_path, request):
    os.environ['ROS_DOMAIN_ID'] = str(226 + os.getpid() % 5)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ.setdefault('ROS_LOG_DIR', str(tmp_path / 'ros'))
    rclpy.init()
    node = rclpy.create_node('phase8_test')
    log = (tmp_path / 'launch.log').open('w+')
    process = subprocess.Popen(
        ['ros2', 'launch', str(SHARE / 'launch/navigation.launch.py'),
         'start_sim:=false', f'cmd_vel_topic:={DEBUG_COMMAND}',
         f'enable:={str(getattr(request, "param", True)).lower()}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        yield Harness(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        log.seek(0)
        output = log.read()
        log.close()
        assert process.returncode == 0, output


def test_amcl_single_goal_reached_and_invalid_goal_stops(navigation):
    h = navigation
    h.wait(lambda: bool(h.outputs))
    h.settle()
    assert all(zero(cmd) for cmd in h.outputs)
    assert not h.buffer.can_transform('map', 'odom', rclpy.time.Time())
    h.initialize()
    h.goal()
    h.wait(lambda: h.status.get('/plan/status') == 'SUCCESS')
    h.wait(lambda: h.status.get('/controller/lqr/status') == 'GOAL_REACHED', seconds=25)
    assert math.hypot(h.pose[0] - 1.325, h.pose[1] - 1.025) < 0.04
    before = len(h.outputs)
    h.settle()
    assert h.outputs[before:] and all(zero(cmd) for cmd in h.outputs[before:])
    assert h.node.count_publishers('/cmd_vel') == 0
    assert h.node.count_publishers(DEBUG_COMMAND) == 1
    h.goal(x=99.0)
    h.wait(lambda: h.status.get('/plan/status') == 'INVALID_GOAL' and
           bool(h.paths) and not h.paths[-1].poses and zero(h.command))
    # Split the synthetic room; legal start/goal remain on opposite sides.
    for y in range(80):
        h.map.data[y * 80 + 40] = 100
    h.maps.publish(h.map)
    h.settle()
    h.goal(x=2.525)
    h.wait(lambda: h.status.get('/plan/status') == 'NO_PATH' and
           not h.paths[-1].poses and zero(h.command))


def test_obstacle_scan_timeout_and_tf_timeout_stop(navigation):
    h = navigation
    h.initialize()
    h.goal(x=2.025)
    h.wait(lambda: not zero(h.command))
    h.obstacle = True
    h.wait(lambda: h.status.get('/controller/safety/status') == 'OBSTACLE_STOP' and
           zero(h.command))
    h.obstacle = False
    h.wait(lambda: not zero(h.command))
    h.emit_scan = False
    h.wait(lambda: h.status.get('/controller/safety/status') == 'SCAN_TIMEOUT' and
           zero(h.command))
    h.emit_scan = True
    h.wait(lambda: not zero(h.command))
    h.emit_tf = False
    h.wait(lambda: h.status.get('/controller/lqr/status') == 'TF_UNAVAILABLE' and
           zero(h.command))
    h.emit_tf = True
    h.wait(lambda: not zero(h.command))
    h.emit_clock = False
    h.wait(lambda: h.status.get('/controller/safety/status') == 'CLOCK_UNAVAILABLE' and
           zero(h.command))


@pytest.mark.parametrize('navigation', [False], indirect=True)
def test_disabled_never_bypasses_gate(navigation):
    h = navigation
    h.initialize()
    h.goal()
    h.wait(lambda: h.status.get('/plan/status') == 'SUCCESS' and
           any(not zero(cmd) for cmd in h.raw_commands))
    h.settle()
    assert h.outputs and all(zero(cmd) for cmd in h.outputs)

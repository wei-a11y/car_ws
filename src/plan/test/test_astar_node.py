# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from contextlib import contextmanager
import math
import os
import signal
import subprocess
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def spin_until(node, predicate, timeout=8.0):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), 'Timed out waiting for ROS discovery or planner output'


def settle(node, seconds=0.3):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def command(*overrides):
    return [os.environ['ASTAR_EXECUTABLE'], '--ros-args', '--params-file',
            os.environ['ASTAR_CONFIG'], '-p', 'planning_frame:=grid_debug',
            '-p', 'use_sim_time:=false', *overrides]


@contextmanager
def running(command_line):
    process = subprocess.Popen(command_line, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        yield process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        assert process.returncode == 0, output.decode()


@pytest.fixture
def node():
    os.environ['ROS_DOMAIN_ID'] = str(25 + os.getpid() % 170)
    rclpy.init()
    value = rclpy.create_node('astar_test')
    try:
        yield value
    finally:
        value.destroy_node()
        rclpy.shutdown()


def map_message():
    msg = OccupancyGrid()
    msg.header.frame_id = 'grid_debug'
    msg.info.width = msg.info.height = 9
    msg.info.resolution = 1.0
    msg.info.origin.orientation.w = 1.0
    msg.data = [0] * 81
    return msg


def transform(child, x, y):
    msg = TransformStamped()
    msg.header.frame_id = 'grid_debug'
    msg.child_frame_id = child
    msg.transform.translation.x = float(x)
    msg.transform.translation.y = float(y)
    msg.transform.rotation.w = 1.0
    return msg


class Harness:
    def __init__(self, node):
        self.node = node
        self.paths, self.raw_paths, self.statuses = [], [], []
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.maps = node.create_publisher(OccupancyGrid, '/plan/inflated_grid', latched)
        self.goals = node.create_publisher(PoseStamped, '/goal_pose', 1)
        node.create_subscription(Path, '/plan', self.paths.append, 1)
        node.create_subscription(Path, '/plan/raw_path', self.raw_paths.append, 1)
        node.create_subscription(String, '/plan/status', self.statuses.append, latched)
        spin_until(node, lambda: self.maps.get_subscription_count() > 0 and
                   self.goals.get_subscription_count() > 0 and
                   node.count_publishers('/plan/raw_path') > 0)

    def send_map(self, msg):
        self.maps.publish(msg)
        settle(self.node)

    def request(self, expected, x=7.5, y=7.5, frame='grid_debug'):
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.pose.position.x, goal.pose.position.y = x, y
        goal.pose.orientation.w = 1.0
        before_status, before_path = len(self.statuses), len(self.paths)
        before_raw = len(self.raw_paths)
        self.goals.publish(goal)
        spin_until(self.node, lambda: len(self.statuses) > before_status and
                   len(self.paths) > before_path and len(self.raw_paths) > before_raw)
        assert self.statuses[-1].data == expected
        path = self.paths[-1]
        assert path.header.frame_id == 'grid_debug'
        assert bool(path.poses) == (expected == 'SUCCESS')
        assert all(p.header == path.header for p in path.poses)
        raw = self.raw_paths[-1]
        assert raw.header == path.header
        assert bool(raw.poses) == (expected == 'SUCCESS')
        assert all(p.header == raw.header for p in raw.poses)
        if path.poses:
            assert path.poses[0].pose.position == raw.poses[0].pose.position
            assert path.poses[-1].pose.position == raw.poses[-1].pose.position
        return path


def test_all_six_statuses_and_failure_clears_path(node):
    with running(command()):
        harness = Harness(node)
        harness.request('MAP_UNAVAILABLE')
        msg = map_message()
        harness.send_map(msg)
        harness.request('TF_UNAVAILABLE')
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform('base_footprint', 1.5, 1.5))
        settle(node)
        path = harness.request('SUCCESS')
        assert path.poses[0].pose.position.x == 1.5
        assert path.poses[-1].pose.position.x == 7.5
        harness.request('INVALID_GOAL', x=99.0)
        harness.request('TF_UNAVAILABLE', frame='missing_goal_frame')

        msg.data[1 * 9 + 1] = 100
        harness.send_map(msg)
        harness.request('INVALID_START')
        msg.data[1 * 9 + 1] = 0
        msg.data[7 * 9 + 7] = 100
        harness.send_map(msg)
        harness.request('INVALID_GOAL')
        msg.data[7 * 9 + 7] = 0
        for row in range(9):
            msg.data[row * 9 + 4] = 100
        harness.send_map(msg)
        harness.request('NO_PATH')
        msg.header.frame_id = 'wrong_map_frame'
        harness.send_map(msg)
        harness.request('MAP_UNAVAILABLE')


def test_rotated_phase2_pipeline_and_goal_frame_transform(node):
    grid_command = [os.environ['PLANNING_GRID_EXECUTABLE'], '--ros-args', '--params-file',
                    os.environ['PLANNING_GRID_CONFIG'], '-p', 'use_sim_time:=false']
    with running(grid_command), running(command()):
        harness = Harness(node)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        raw = node.create_publisher(OccupancyGrid, '/map', latched)
        inflated = []
        node.create_subscription(OccupancyGrid, '/plan/inflated_grid', inflated.append, latched)
        spin_until(node, lambda: raw.get_subscription_count() > 0)
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform([
            transform('base_footprint', 1.25, -0.25), transform('goal_debug', 10.0, -5.0)])
        msg = map_message()
        msg.info.resolution = 0.5
        msg.info.origin.position.x, msg.info.origin.position.y = 2.0, -1.0
        msg.info.origin.orientation.z = msg.info.origin.orientation.w = math.sqrt(0.5)
        msg.data[4 * 9 + 4] = 100
        raw.publish(msg)
        spin_until(node, lambda: bool(inflated))
        settle(node)
        path = harness.request('SUCCESS', x=-11.75, y=7.75, frame='goal_debug')
        assert len(path.poses) > 7  # The obstacle forces a detour.
        assert path.poses[0].pose.position.x == pytest.approx(1.25)
        assert path.poses[0].pose.position.y == pytest.approx(-0.25)
        assert path.poses[-1].pose.position.x == pytest.approx(-1.75)
        assert path.poses[-1].pose.position.y == pytest.approx(2.75)
        for pose in path.poses:
            # Invert the synthetic 90-degree origin and verify the actual Phase 2 mask.
            x = math.floor((pose.pose.position.y + 1.0) / 0.5)
            y = math.floor((2.0 - pose.pose.position.x) / 0.5)
            assert inflated[-1].data[y * 9 + x] != 100
            q = pose.pose.orientation
            assert q.z * q.z + q.w * q.w == pytest.approx(1.0)
        for debug_path in (path, harness.raw_paths[-1]):
            for a, b in zip(debug_path.poses, debug_path.poses[1:]):
                assert segment_free(inflated[-1], a.pose.position, b.pose.position)


def test_stale_tf_is_rejected_and_start_is_refreshed(node):
    with running(command()):
        harness = Harness(node)
        harness.send_map(map_message())
        broadcaster = TransformBroadcaster(node)
        robot = transform('base_footprint', 1.5, 1.5)
        robot.header.stamp = node.get_clock().now().to_msg()
        robot.header.stamp.sec -= 10
        broadcaster.sendTransform(robot)
        settle(node)
        harness.request('TF_UNAVAILABLE')
        robot.header.stamp = node.get_clock().now().to_msg()
        broadcaster.sendTransform(robot)
        settle(node, 0.05)
        path = harness.request('SUCCESS')
        assert path.poses[0].pose.position.x == 1.5
        robot.transform.translation.x = 3.5
        robot.header.stamp = node.get_clock().now().to_msg()
        broadcaster.sendTransform(robot)
        settle(node, 0.05)
        path = harness.request('SUCCESS')
        assert path.poses[0].pose.position.x == 3.5


def test_explicit_planning_frame_is_required(node):
    result = subprocess.run(command('-p', 'planning_frame:='), capture_output=True, timeout=8.0)
    assert result.returncode != 0
    assert 'planning_frame' in (result.stdout + result.stderr).decode()


def segment_free(grid, start, end):
    q = grid.info.origin.orientation
    yaw = 2.0 * math.atan2(q.z, q.w)

    def local(point):
        dx = point.x - grid.info.origin.position.x
        dy = point.y - grid.info.origin.position.y
        return ((math.cos(yaw) * dx + math.sin(yaw) * dy) / grid.info.resolution,
                (-math.sin(yaw) * dx + math.cos(yaw) * dy) / grid.info.resolution)

    a, b = local(start), local(end)
    for index, occupancy in enumerate(grid.data):
        if occupancy != 100:
            continue
        lower = (index % grid.info.width, index // grid.info.width)
        enter, leave = 0.0, 1.0
        for axis in (0, 1):
            delta = b[axis] - a[axis]
            if abs(delta) < 1e-12:
                if not lower[axis] - 1e-10 <= a[axis] <= lower[axis] + 1.0 + 1e-10:
                    break
            else:
                first = (lower[axis] - a[axis]) / delta
                last = (lower[axis] + 1.0 - a[axis]) / delta
                enter = max(enter, min(first, last))
                leave = min(leave, max(first, last))
                if enter > leave + 1e-10:
                    break
        else:
            return False
    return True


@pytest.mark.parametrize('shortcut', [True, False])
def test_raw_and_processed_spacing_yaw_and_map_update_clear(node, shortcut):
    with running(command('-p', f'post_processing.enable_shortcut:={str(shortcut).lower()}',
                         '-p', 'post_processing.resample_spacing:=0.5')):
        harness = Harness(node)
        msg = map_message()
        msg.data[4 * 9 + 4] = 100
        harness.send_map(msg)
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform('base_footprint', 1.5, 1.5))
        settle(node)
        path = harness.request('SUCCESS')
        raw = harness.raw_paths[-1]
        assert len(raw.poses) < len(path.poses)
        for pose in raw.poses:
            assert pose.pose.position.x % 1.0 == pytest.approx(0.5)
            assert pose.pose.position.y % 1.0 == pytest.approx(0.5)
        for a, b in zip(path.poses, path.poses[1:]):
            dx = b.pose.position.x - a.pose.position.x
            dy = b.pose.position.y - a.pose.position.y
            assert 0.0 < math.hypot(dx, dy) <= 0.5 + 1e-10
            yaw = math.atan2(dy, dx)
            assert a.pose.orientation.z == pytest.approx(math.sin(yaw / 2.0))
            assert a.pose.orientation.w == pytest.approx(math.cos(yaw / 2.0))
            assert segment_free(msg, a.pose.position, b.pose.position)
        assert path.poses[-1].pose.orientation == path.poses[-2].pose.orientation
        before_path, before_raw = len(harness.paths), len(harness.raw_paths)
        harness.send_map(msg)
        spin_until(node, lambda: len(harness.paths) > before_path and
                   len(harness.raw_paths) > before_raw)
        assert not harness.paths[-1].poses
        assert not harness.raw_paths[-1].poses
        harness.request('SUCCESS')


def test_processing_limit_failure_clears_both_outputs(node):
    with running(command('-p', 'post_processing.max_output_points:=2')):
        harness = Harness(node)
        harness.send_map(map_message())
        broadcaster = StaticTransformBroadcaster(node)
        broadcaster.sendTransform(transform('base_footprint', 1.5, 1.5))
        settle(node)
        harness.request('NO_PATH')


@pytest.mark.parametrize('override', [
    'post_processing.resample_spacing:=0.0',
    'post_processing.shortcut_max_lookahead:=0',
    'post_processing.max_output_points:=0',
    'raw_path_topic:=/plan',
])
def test_invalid_processing_config_or_aliased_output_rejected(node, override):
    result = subprocess.run(command('-p', override), capture_output=True, timeout=8.0)
    assert result.returncode != 0
    assert 'Startup failed' in (result.stdout + result.stderr).decode()

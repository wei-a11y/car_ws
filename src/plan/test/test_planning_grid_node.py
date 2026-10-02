# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
import math
import os
import signal
import subprocess
import time

from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message


def command(*overrides):
    return [
        os.environ['PLANNING_GRID_EXECUTABLE'], '--ros-args', '--params-file',
        os.environ['PLANNING_GRID_CONFIG'], '-p', 'use_sim_time:=false', *overrides,
    ]


@pytest.fixture
def ros_environment():
    # A separate localhost-only domain; this node never publishes velocity commands.
    os.environ['ROS_DOMAIN_ID'] = str(20 + os.getpid() % 200)
    rclpy.init()
    node = rclpy.create_node('grid_test')
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def spin_until(node, predicate, timeout=8.0):
    end = time.monotonic() + timeout
    while not predicate() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    assert predicate(), 'Timed out waiting for grid node/discovery/output'


def test_maps_metadata_rejection_and_late_joiner(ros_environment):
    node = ros_environment
    process = subprocess.Popen(command(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        raw, inflated = [], []
        publisher = node.create_publisher(OccupancyGrid, '/map', qos)
        node.create_subscription(OccupancyGrid, '/plan/raw_grid', raw.append, qos)
        node.create_subscription(OccupancyGrid, '/plan/inflated_grid', inflated.append, qos)
        spin_until(node, lambda: publisher.get_subscription_count() > 0)
        msg = OccupancyGrid()
        msg.header.frame_id = 'grid_debug'
        msg.header.stamp.sec = 123
        msg.info.width = msg.info.height = 21
        msg.info.resolution = 0.1
        msg.info.origin.position.x = 10.0
        msg.info.origin.position.y = -3.0
        msg.info.origin.orientation.z = msg.info.origin.orientation.w = math.sqrt(0.5)
        msg.data = [0] * 441
        msg.data[10 * 21 + 10] = 65
        msg.data[5 * 21 + 5] = -1
        msg.data[10 * 21 + 15] = 64
        # MapMetaData.resolution is float32 on the wire, even though Python uses float64.
        expected = deserialize_message(serialize_message(msg), OccupancyGrid)
        publisher.publish(msg)
        spin_until(node, lambda: bool(raw) and bool(inflated))
        assert raw[-1] == expected
        assert inflated[-1].header == expected.header
        assert inflated[-1].info == expected.info
        assert inflated[-1].data[10 * 21 + 14] == 100
        assert inflated[-1].data[10 * 21 + 15] == 0
        assert inflated[-1].data[5 * 21 + 5] == 100
        assert inflated[-1].data[0] == 100  # The map boundary protects the robot footprint.

        # Invalid input must not be published as a fresh successful grid.
        msg.info.resolution = 0.0
        msg.header.stamp.sec = 124
        publisher.publish(msg)
        until = time.monotonic() + 0.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        late = []
        node.create_subscription(OccupancyGrid, '/plan/inflated_grid', late.append, qos)
        spin_until(node, lambda: bool(late))
        assert late[-1].header.stamp.sec == 123
        assert late[-1].info.resolution > 0.0
        assert process.poll() is None
    finally:
        process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        assert process.returncode == 0, output.decode()


@pytest.mark.parametrize('overrides,diagnostic', [
    (['-p', 'robot_radius:=0.0'], 'Invalid threshold'),
    (['-r', '/plan/raw_grid:=/map'], 'must be distinct'),
])
def test_invalid_config_fails_startup(ros_environment, overrides, diagnostic):
    result = subprocess.run(command(*overrides), capture_output=True, timeout=8.0)
    assert result.returncode != 0
    assert diagnostic in (result.stdout + result.stderr).decode()

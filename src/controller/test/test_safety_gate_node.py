# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from contextlib import contextmanager
import math
import os
import signal
import subprocess
import time

from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, String
from tf2_ros import StaticTransformBroadcaster


def command(*overrides):
    # Even within the isolated DDS domain, NEVER output to a real base command topic.
    return [os.environ['GATE_EXECUTABLE'], '--ros-args', '--params-file',
            os.environ['GATE_CONFIG'], '-p', 'use_sim_time:=false',
            '-r', '/cmd_vel_raw:=/gate_test/raw', '-r', '/cmd_vel:=/gate_test/out',
            '-r', '/scan:=/gate_test/scan',
            '-r', '/controller/safety/status:=/gate_test/status',
            '-r', '/controller/safety/min_range:=/gate_test/min_range', *overrides]


@contextmanager
def running(arguments):
    process = subprocess.Popen(arguments, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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


def spin_until(node, predicate, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not predicate():
        rclpy.spin_once(node, timeout_sec=0.01)
    assert predicate(), 'Timed out waiting for Safety Gate'


def settle(node, seconds=0.15):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


@pytest.fixture(autouse=True)
def isolated_domain():
    # Also isolate parameter-rejection tests when they are selected individually.
    # Disjoint from plan domains; controller ROS tests run sequentially in CTest.
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 25)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'


@pytest.fixture
def node():
    rclpy.init()
    value = rclpy.create_node('safety_gate_test')
    try:
        yield value
    finally:
        value.destroy_node()
        rclpy.shutdown()


def zero(message):
    values = (message.linear.x, message.linear.y, message.linear.z,
              message.angular.x, message.angular.y, message.angular.z)
    return all(v == 0.0 for v in values)


class Harness:
    def __init__(self, node, tf=True, simulated_clock=False):
        self.node = node
        self.outputs, self.statuses, self.distances = [], [], []
        self.emit_scan, self.emit_raw = True, True
        self.range = 8.0
        self.scan_frame = 'laser_link'
        self.stamp_offset = 0
        self.raw = Twist()
        self.raw.linear.x, self.raw.angular.z = 0.15, 0.2
        self.clock_ns = 1000000000 if simulated_clock else None
        self.emit_clock = simulated_clock
        self.raw_pub = node.create_publisher(Twist, '/gate_test/raw', 1)
        sensors = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_pub = node.create_publisher(LaserScan, '/gate_test/scan', sensors)
        self.clock_pub = node.create_publisher(Clock, '/clock', 1) if simulated_clock else None
        node.create_subscription(Twist, '/gate_test/out', self.outputs.append, 1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/gate_test/status', self.statuses.append, latched)
        node.create_subscription(Float64, '/gate_test/min_range', self.distances.append, 1)
        self.static_tf = StaticTransformBroadcaster(node)
        if tf:
            transform = TransformStamped()
            transform.header.frame_id = 'base_footprint'
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.child_frame_id = 'laser_link'
            transform.transform.translation.x = 0.155
            transform.transform.translation.z = 0.2835
            transform.transform.rotation.z = math.sin(3.14 / 2.0)
            transform.transform.rotation.w = math.cos(3.14 / 2.0)
            self.static_tf.sendTransform(transform)
        self.timer = node.create_timer(0.03, self.tick)
        spin_until(node, lambda: self.raw_pub.get_subscription_count() > 0 and
                   self.scan_pub.get_subscription_count() > 0)

    def tick(self):
        if self.clock_ns is None:
            stamp = self.node.get_clock().now().to_msg()
        else:
            if self.emit_clock:
                self.clock_ns += 30000000
                clock_stamp = Time(sec=self.clock_ns // 1000000000,
                                   nanosec=self.clock_ns % 1000000000)
                self.clock_pub.publish(Clock(clock=clock_stamp))
            stamp = Time(sec=self.clock_ns // 1000000000, nanosec=self.clock_ns % 1000000000)
        if self.emit_raw:
            self.raw_pub.publish(self.raw)
        if self.emit_scan:
            scan = LaserScan()
            scan.header.frame_id = self.scan_frame
            scan.header.stamp = Time(sec=stamp.sec + self.stamp_offset, nanosec=stamp.nanosec)
            scan.angle_min, scan.angle_max = 0.349, 5.934
            scan.angle_increment = (scan.angle_max - scan.angle_min) / 359
            scan.range_min, scan.range_max = 0.12, 8.0
            scan.ranges = [float(self.range)] * 360
            self.scan_pub.publish(scan)

    def wait(self, code, moving=False):
        spin_until(self.node, lambda: bool(self.statuses) and self.statuses[-1].data == code and
                   bool(self.outputs) and zero(self.outputs[-1]) != moving)


def test_clear_stop_hysteresis_release_and_zero_raw(node):
    with running(command()):
        h = Harness(node)
        h.wait('CLEAR', moving=True)
        assert h.outputs[-1].linear.x == pytest.approx(0.15)
        assert h.outputs[-1].angular.z == pytest.approx(0.2)
        h.range = 0.4
        h.wait('OBSTACLE_STOP')
        h.range = 0.6
        settle(node, 0.3)
        h.wait('OBSTACLE_STOP')
        h.range = 0.7
        h.wait('CLEAR', moving=True)
        h.range = 0.6
        settle(node)
        h.wait('CLEAR', moving=True)
        h.raw = Twist()
        spin_until(node, lambda: bool(h.outputs) and zero(h.outputs[-1]))


def test_scan_and_raw_timeout_and_recovery(node):
    with running(command()):
        h = Harness(node)
        h.wait('CLEAR', moving=True)
        h.emit_scan = False
        h.wait('SCAN_TIMEOUT')
        h.range, h.emit_scan = 0.6, True
        h.wait('OBSTACLE_STOP')
        h.range = 8.0
        h.wait('CLEAR', moving=True)
        h.emit_raw = False
        h.wait('RAW_TIMEOUT')
        h.emit_raw = True
        h.wait('CLEAR', moving=True)


@pytest.mark.parametrize('bad_range', [float('nan'), float('-inf'), 0.1, 8.1])
def test_invalid_scan_stops_and_requires_valid_scan(node, bad_range):
    with running(command()):
        h = Harness(node)
        h.wait('CLEAR', moving=True)
        h.range = bad_range
        h.wait('SCAN_INVALID')
        h.range = 0.6
        h.wait('OBSTACLE_STOP')
        h.range = 8.0
        h.wait('CLEAR', moving=True)


def test_positive_infinity_no_return(node):
    with running(command()):
        h = Harness(node)
        h.range = float('inf')
        h.wait('CLEAR', moving=True)
        spin_until(node, lambda: bool(h.distances) and h.distances[-1].data == 8.0)


def test_wrong_frame_old_and_future_scan_stamp(node):
    with running(command()):
        h = Harness(node)
        h.wait('CLEAR', moving=True)
        h.scan_frame = 'unverified_laser'
        h.wait('SCAN_INVALID')
        h.scan_frame, h.stamp_offset = 'laser_link', -5
        h.wait('SCAN_TIMEOUT')
        h.stamp_offset = 5
        h.wait('SCAN_TIMEOUT')
        h.stamp_offset = 0
        h.wait('CLEAR', moving=True)


def test_missing_tf_fails_closed(node):
    with running(command()):
        h = Harness(node, tf=False)
        h.wait('TF_UNAVAILABLE')
        settle(node, 0.4)
        assert all(zero(message) for message in h.outputs)


def test_invalid_nonplanar_reverse_and_overspeed_raw(node):
    with running(command()):
        h = Harness(node)
        h.wait('CLEAR', moving=True)
        for field, bad in [('linear.x', -0.1), ('linear.x', 0.3),
                           ('angular.z', 0.7), ('linear.y', 0.1), ('linear.x', float('nan'))]:
            h.raw = Twist()
            vector, axis = field.split('.')
            setattr(getattr(h.raw, vector), axis, bad)
            h.wait('RAW_INVALID')
            h.raw = Twist()
            h.raw.linear.x = 0.15
            h.wait('CLEAR', moving=True)


def test_disabled_is_zero_not_passthrough(node):
    with running(command('-p', 'enable:=false')):
        h = Harness(node)
        h.wait('DISABLED')
        settle(node, 0.4)
        assert all(zero(message) for message in h.outputs)


def test_paused_clock_and_clock_reset_stop(node):
    with running(command('-p', 'use_sim_time:=true')):
        h = Harness(node, simulated_clock=True)
        h.wait('CLEAR', moving=True)
        h.emit_clock = False  # Inputs still arrive with frozen timestamps.
        h.wait('CLOCK_UNAVAILABLE')
        h.emit_clock = True
        h.wait('CLEAR', moving=True)
        h.emit_scan, h.emit_raw = False, False
        h.clock_ns = 500000000
        spin_until(node, lambda: bool(h.outputs) and zero(h.outputs[-1]) and
                   any(message.data == 'CLOCK_RESET' for message in h.statuses))
        h.emit_scan, h.emit_raw = True, True
        h.wait('CLEAR', moving=True)


@pytest.mark.parametrize('override', [
    'stop_distance:=0.7', 'max_beam_gap:=0.0', 'scan_timeout_sec:=0.01', 'v_max:=-1.0'])
def test_invalid_parameters_rejected(override):
    process = subprocess.run(command('-p', override), stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, timeout=10.0)
    assert process.returncode != 0


def test_remapped_raw_output_collision_rejected():
    process = subprocess.run(command('-p', 'raw_cmd_topic:=/gate_test/out'),
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10.0)
    assert process.returncode != 0

# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
from contextlib import contextmanager
import math
import os
from pathlib import Path as FilePath
import signal
import subprocess
import time

from builtin_interfaces.msg import Time
from common_msg.msg import PathProfile
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster


def command(*overrides):
    return [os.environ['TRACKER_EXECUTABLE'], '--ros-args', '--params-file',
            os.environ['TRACKER_CONFIG'], '--params-file',
            str(FilePath(__file__).parents[2] / 'plan/config/motion_limits.yaml'),
            '-p', 'planning_frame:=control_debug',
            '-p', 'use_sim_time:=false', '-p', 'require_path_profile:=false', *overrides]


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
        process.output = output.decode()
        assert process.returncode == 0, output.decode()


def spin_until(node, predicate, timeout=8.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end and not predicate():
        rclpy.spin_once(node, timeout_sec=0.02)
    assert predicate(), 'Timed out waiting for tracker output'


def settle(node, seconds=0.12):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.01)


@pytest.fixture
def node():
    # Disjoint from the plan test domain range, including parallel colcon test runs.
    os.environ['ROS_DOMAIN_ID'] = str(200 + os.getpid() % 25)
    rclpy.init()
    value = rclpy.create_node('lqr_tracker_test')
    try:
        yield value
    finally:
        value.destroy_node()
        rclpy.shutdown()


def path_message(points=((0.0, 0.0), (1.0, 0.0)), frame='control_debug'):
    message = Path()
    message.header.frame_id = frame
    message.header.stamp.sec = 1  # Deliberately old: Path is not a heartbeat.
    for x, y in points:
        pose = PoseStamped()
        pose.header = message.header
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
    return message


def zero(message):
    return message.linear.x == 0.0 and message.angular.z == 0.0


class Harness:
    def __init__(self, node, simulated_clock=False):
        self.node = node
        self.commands, self.statuses, self.debug = [], [], []
        self.pose = [0.0, 0.0, 0.0]
        self.actual = [0.0, 0.0]
        self.emit_tf, self.emit_odom = True, True
        self.odom_child = 'base_footprint'
        self.stamp_offset = 0
        self.clock_ns = 1000000000 if simulated_clock else None
        self.emit_clock = simulated_clock
        self.paths = node.create_publisher(Path, '/plan', 1)
        self.profiles = node.create_publisher(PathProfile, '/plan/profile', 1)
        sensors = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odometry = node.create_publisher(Odometry, '/odom', sensors)
        self.clock = node.create_publisher(Clock, '/clock', 1) if simulated_clock else None
        self.tf = TransformBroadcaster(node)
        node.create_subscription(Twist, '/cmd_vel_raw', self.commands.append, 1)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/controller/lqr/status', self.statuses.append, latched)
        node.create_subscription(Float64MultiArray, '/controller/lqr/debug', self.debug.append, 1)
        self.timer = node.create_timer(0.05 if simulated_clock else 0.02, self.tick)
        spin_until(node, lambda: self.paths.get_subscription_count() > 0 and
                   self.odometry.get_subscription_count() > 0)
        settle(node)

    def tick(self):
        if self.clock_ns is None:
            stamp = self.node.get_clock().now().to_msg()
        else:
            if self.emit_clock:
                self.clock_ns += 50000000
                clock_stamp = Time(sec=self.clock_ns // 1000000000,
                                   nanosec=self.clock_ns % 1000000000)
                self.clock.publish(Clock(clock=clock_stamp))
            stamp = Time(sec=self.clock_ns // 1000000000, nanosec=self.clock_ns % 1000000000)
        if self.emit_odom:
            message = Odometry()
            message.header.frame_id = 'odom'
            message.header.stamp = Time(sec=stamp.sec + self.stamp_offset, nanosec=stamp.nanosec)
            message.child_frame_id = self.odom_child
            message.pose.pose.position.x = 999.0  # Must never replace the TF pose.
            message.twist.twist.linear.x, message.twist.twist.angular.z = self.actual
            self.odometry.publish(message)
        if self.emit_tf:
            message = TransformStamped()
            message.header.frame_id = 'control_debug'
            message.header.stamp = stamp
            message.child_frame_id = 'base_footprint'
            message.transform.translation.x, message.transform.translation.y = self.pose[:2]
            message.transform.rotation.z = math.sin(self.pose[2] / 2.0)
            message.transform.rotation.w = math.cos(self.pose[2] / 2.0)
            self.tf.sendTransform(message)

    def send_path(self, message=None):
        self.paths.publish(path_message() if message is None else message)
        settle(self.node)

    def wait(self, code, nonzero=False):
        spin_until(self.node, lambda: bool(self.statuses) and self.statuses[-1].data == code and
                   bool(self.commands) and zero(self.commands[-1]) != nonzero)


def test_tracks_with_tf_pose_old_path_and_stops_reliably_at_goal(node):
    with running(command()):
        h = Harness(node)
        h.wait('WAIT_PATH')
        h.send_path()
        h.wait('TRACKING', nonzero=True)
        assert h.commands[-1].linear.x == pytest.approx(0.15)
        assert h.debug[-1].data[7] == pytest.approx(0.0)  # TF, not odometry.pose.x=999.
        h.pose[0] = 0.5
        settle(node)
        h.pose[0] = 0.9
        settle(node)
        h.wait('TRACKING', nonzero=True)
        assert h.commands[-1].linear.x == pytest.approx(0.1)
        h.actual[0] = 0.04
        h.pose[0] = 0.98
        h.wait('BRAKING')
        h.actual[0] = 0.0
        h.wait('GOAL_REACHED')
        before = len(h.commands)
        settle(node, 0.4)
        assert len(h.commands) > before + 3
        assert all(zero(msg) for msg in h.commands[before:])
        assert all(msg.linear.y == 0.0 and msg.linear.z == 0.0 and
                   msg.angular.x == 0.0 and msg.angular.y == 0.0 for msg in h.commands)


def test_empty_invalid_path_and_wrong_odom_frame_stop_old_command(node):
    with running(command()) as process:
        h = Harness(node)
        h.send_path()
        h.wait('TRACKING', nonzero=True)
        h.send_path(path_message(frame='wrong_frame'))
        h.wait('PATH_INVALID')
        h.send_path()
        h.wait('TRACKING', nonzero=True)
        h.odom_child = 'wrong_base'
        h.wait('ODOM_UNAVAILABLE')
        h.odom_child = 'base_footprint'
        h.wait('TRACKING', nonzero=True)
        h.send_path(path_message(points=()))
        h.wait('PATH_UNAVAILABLE')
    assert 'stage=PATH_FRAME received=wrong_frame expected=control_debug' in process.output
    assert 'stage=ODOM_FRAME received=odom->wrong_base' in process.output


def test_stale_missing_tf_odom_and_disappearing_path_publisher_stop(node):
    with running(command('-p', 'tf_max_age_sec:=0.2', '-p', 'odom_max_age_sec:=0.15')) as process:
        h = Harness(node)
        h.emit_tf = False
        settle(node, 0.3)
        h.send_path()
        h.wait('TF_UNAVAILABLE')
        h.emit_tf = True
        h.wait('TRACKING', nonzero=True)
        h.emit_odom = False
        h.wait('ODOM_UNAVAILABLE')
        h.emit_odom = True
        h.wait('TRACKING', nonzero=True)
        h.stamp_offset = 30
        h.wait('ODOM_UNAVAILABLE')
        h.stamp_offset = 0
        h.wait('TRACKING', nonzero=True)
        node.destroy_publisher(h.paths)
        h.wait('PATH_UNAVAILABLE')
    assert 'stage=CURRENT_POSE target=control_debug source=base_footprint' in process.output
    assert 'stage=ODOM_TIME signed_age=' in process.output
    assert 'stage=PATH_PUBLISHER reason=PUBLISHER_DISAPPEARED' in process.output


def test_fault_log_names_guard_keeps_snapshot_and_is_rate_limited(node):
    with running(command('-p', 'diagnostic_log_period_sec:=0.2')) as process:
        h = Harness(node)
        h.send_path()
        h.wait('TRACKING', nonzero=True)
        h.pose[1] = 0.05
        h.wait('TRACKING_ERROR')
        h.pose[1] = 0.0  # A recovered pose must not overwrite the original fault.
        settle(node, 0.7)
        h.wait('TRACKING_ERROR')
        assert len(h.debug[-1].data) == 11  # Existing interface is unchanged.
    faults = [line for line in process.output.splitlines()
              if 'TRACKING_ERROR: stage=CORE_TRACKER' in line]
    assert 2 <= len(faults) <= 8
    assert all('reason=NEAREST_DISTANCE_EXCEEDED' in line for line in faults)
    assert all('snapshot=first_fault_latched' in line for line in faults)
    assert all('nearest_distance=0.050000 limit=0.030000' in line for line in faults)
    assert all('pose=(0.000000,0.050000,0.000000)' in line for line in faults)
    assert all('segment=0 segment_count=1' in line for line in faults)


def test_feedback_saturation_and_debug_layout(node):
    with running(command('-p', 'w_max:=0.05')):
        h = Harness(node)
        h.pose[1] = 0.02
        h.send_path()
        h.wait('TRACKING', nonzero=True)
        assert h.commands[-1].angular.z == pytest.approx(-0.05)
        assert h.debug[-1].data[0] == pytest.approx(0.02)
        assert len(h.debug[-1].data) == 11
        assert all(0.0 <= msg.linear.x <= 0.2 and abs(msg.angular.z) <= 0.05 for msg in h.commands)


def test_simulated_clock_pause_wall_watchdog_publishes_zero(node):
    with running(command('-p', 'use_sim_time:=true', '-p', 'wall_watchdog_timeout_sec:=0.2')):
        h = Harness(node, simulated_clock=True)
        h.send_path()
        h.wait('TRACKING', nonzero=True)
        h.emit_clock = False
        h.wait('CLOCK_UNAVAILABLE')
        before = len(h.commands)
        settle(node, 0.3)
        assert all(zero(msg) for msg in h.commands[before:])


@pytest.mark.parametrize('override', [
    ['-p', 'planning_frame:=""'],
    ['-p', 'v_nominal:=0.0'],
    ['-p', 'q_lateral:=-1.0'],
    ['-p', 'riccati_max_iterations:=1'],
    ['-p', 'diagnostic_log_period_sec:=0.0'],
    ['-p', 'linear_deceleration:=0.0'],
    ['-p', 'feedforward_angular_fraction:=1.0'],
    ['-p', 'reference_heading_tolerance:=2.0'],
    ['-p', 'max_reference_curvature:=-1.0'],
    ['-p', 'profile_topic:=/plan'],
    ['-p', 'allow_odom_fallback:=true'],
    ['-p', 'raw_cmd_topic:=/cmd_vel'],
    ['-r', '/cmd_vel_raw:=/diff_drive_controller/cmd_vel_unstamped'],
])
def test_invalid_model_frames_or_base_output_rejected(node, override):
    result = subprocess.run(command(*override), capture_output=True, timeout=8.0)
    assert result.returncode != 0
    assert 'Startup failed' in (result.stdout + result.stderr).decode()


def straight_profile(path):
    message = PathProfile()
    message.path = path
    message.tangent_yaw = [0.0] * len(path.poses)
    message.curvature = [0.0] * len(path.poses)
    message.stop_required = [False] * len(path.poses)
    if path.poses:
        message.stop_required[-1] = True
    return message


def test_continuous_profile_exact_pairing_invalid_arrays_and_publisher_loss(node):
    with running(command('-p', 'require_path_profile:=true')):
        h = Harness(node)
        path = path_message()
        h.send_path(path)
        h.wait('WAIT_PATH_PROFILE')
        wrong = straight_profile(path_message())
        wrong.path.header.stamp.sec = 2
        h.profiles.publish(wrong)
        h.wait('WAIT_PATH_PROFILE')
        h.profiles.publish(straight_profile(path))
        h.wait('TRACKING', nonzero=True)
        replacement = path_message(points=((0.0, 0.0), (2.0, 0.0)))
        h.send_path(replacement)  # Same stamp, different geometry must not pair.
        h.wait('WAIT_PATH_PROFILE')
        invalid = straight_profile(replacement)
        invalid.curvature = []
        h.profiles.publish(invalid)
        h.wait('PATH_PROFILE_INVALID')
        partial = straight_profile(replacement)
        partial.speed_limit = [.1, 0.]
        h.profiles.publish(partial)
        h.wait('PATH_PROFILE_INVALID')
        extended = straight_profile(replacement)
        extended.segment_type = [0, 0]
        extended.speed_limit = [.1, 0.]
        extended.curvature_bound = [0., 0.]
        extended.curvature_rate_bound = [0., 0.]
        h.profiles.publish(extended)
        h.wait('TRACKING', nonzero=True)
        h.profiles.publish(straight_profile(replacement))
        h.wait('TRACKING', nonzero=True)
        node.destroy_publisher(h.profiles)
        h.wait('PATH_UNAVAILABLE')


def test_profile_can_arrive_before_path_and_circular_samples_do_not_stop(node):
    with running(command('-p', 'require_path_profile:=true')):
        h = Harness(node)
        radius = 0.4
        points = [(radius * math.sin(i * math.pi / 40),
                   radius * (1.0 - math.cos(i * math.pi / 40))) for i in range(21)]
        points.append((radius, radius + 1.0))
        path = path_message(points=points)
        profile = PathProfile()
        profile.path = path
        profile.tangent_yaw = [i * math.pi / 40 for i in range(21)] + [math.pi / 2]
        profile.curvature = [1.0 / radius] * 20 + [0.0, 0.0]
        profile.stop_required = [False] * 21 + [True]
        h.profiles.publish(profile)
        settle(node)
        h.wait('WAIT_PATH_PROFILE')
        h.send_path(path)
        h.wait('TRACKING', nonzero=True)
        for theta in (0.1, 0.3, 0.6, 0.9, 1.2, 1.4):
            h.pose = [radius * math.sin(theta), radius * (1.0 - math.cos(theta)), theta]
            h.actual = [0.08, 0.2]
            settle(node)
            h.wait('TRACKING', nonzero=True)
            assert h.commands[-1].linear.x > 0.0
            assert h.commands[-1].angular.z > 0.0


def test_continuous_curvature_and_synchronized_tracking_snapshot(node):
    samples = []
    node.create_subscription(Float64MultiArray, '/controller/lqr/tracking', samples.append, 10)
    with running(command('-p', 'require_path_profile:=true')):
        h = Harness(node)
        h.pose[0] = .01
        path = path_message(points=((0., 0.), (.02, 0.), (.04, .00016), (2., .03)))
        profile = PathProfile(path=path, tangent_yaw=[0., .004, .016, .016],
                              curvature=[0., .4, .8, 0.],
                              stop_required=[False, False, False, True],
                              curvature_continuous=True)
        h.send_path(path)
        h.profiles.publish(profile)
        h.wait('TRACKING', nonzero=True)
        spin_until(node, lambda: bool(samples) and bool(h.debug) and samples[-1].data[5] > 0.)
        fields = samples[-1].layout.dim[0].label.split(',')
        assert len(fields) == len(samples[-1].data)
        data = dict(zip(fields, samples[-1].data))
        assert data['curvature'] == pytest.approx(.6, abs=.001)
        assert data['w_ff'] == pytest.approx(data['v_cmd'] * data['curvature'])
        assert data['pose_x'] == pytest.approx(.01)
        assert data['v_actual'] == 0.
        assert data['path_sec'] == path.header.stamp.sec
        assert data['path_nanosec'] == path.header.stamp.nanosec
        assert data['geometry_valid'] == 1.
        assert data['state'] == 2.
        assert len(h.debug[-1].data) == 11

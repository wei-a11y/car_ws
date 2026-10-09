# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Exercise the real task node with ROS peers, not mocks of its callbacks."""
from copy import deepcopy
import json
import math
import os
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster
import yaml

from task_runtime.node import TaskManager, stamp_ns


def pump(executor, predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not predicate():
        executor.spin_once(timeout_sec=0.01)
    assert predicate(), 'Timed out waiting for ROS task result'


def diagnostic(state, **values):
    return DiagnosticArray(status=[DiagnosticStatus(
        name='test_peer', message=state,
        values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])])


class Peers(Node):
    def __init__(self):
        super().__init__('task_test_peers')
        self.pose = [0.25, 0.25, 0.0]
        self.goal = None
        self.goals, self.events, self.inhibits, self.states = [], [], [], []
        self.auto_reach = True
        self.respond_plan = True
        self.failure = None
        self.lqr_error = None
        self.safety = 'CLEAR'
        self.send_odom = True
        self.actual_v = 0.0
        self.tf = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/plan/inflated_grid', latched)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.reached_pub = self.create_publisher(PoseStamped, '/controller/lqr/reached_goal', 10)
        self.plan_diag = self.create_publisher(DiagnosticArray, '/plan/diagnostics', 10)
        self.lqr_diag = self.create_publisher(DiagnosticArray, '/controller/lqr/diagnostics', 10)
        self.status = {key: self.create_publisher(String, topic, latched) for key, topic in (
            ('lqr', '/controller/lqr/status'), ('planner', '/plan/status'),
            ('safety', '/controller/safety/status'))}
        self.range_pub = self.create_publisher(Float64, '/controller/safety/min_range', 10)
        self.raw_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_service(SetBool, '/controller/lqr/set_inhibit', self.on_inhibit)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(String, '/task/events',
                                 lambda msg: self.events.append(json.loads(msg.data)), 20)
        self.create_subscription(String, '/task/status',
                                 lambda msg: self.states.append(json.loads(msg.data)), latched)
        self.start = self.create_client(Trigger, '/task/start')
        self.timer = self.create_timer(0.02, self.tick)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 100
        grid.info.resolution = 0.1
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 10000
        self.grid_pub.publish(grid)

    def on_inhibit(self, request, response):
        self.inhibits.append(request.data)
        self.goal = None
        response.success = True
        return response

    def on_goal(self, msg):
        self.goals.append(msg)
        self.goal = msg
        if self.failure:
            self.plan_diag.publish(diagnostic(
                self.failure, goal_stamp_ns=stamp_ns(msg.header.stamp),
                path_stamp_ns=0, detail='blocked pickup cell (4, 5)'))
            return
        if not self.respond_plan:
            return
        end = deepcopy(msg)
        end.pose.position.x = (math.floor(msg.pose.position.x / 0.1) + 0.5) * 0.1
        end.pose.position.y = (math.floor(msg.pose.position.y / 0.1) + 0.5) * 0.1
        self.goal = end
        self.path_pub.publish(Path(header=msg.header, poses=[end]))
        self.plan_diag.publish(diagnostic(
            'SUCCESS', goal_stamp_ns=stamp_ns(msg.header.stamp),
            path_stamp_ns=stamp_ns(msg.header.stamp), detail='planned'))

    def tick(self):
        if self.goal and self.auto_reach and self.respond_plan and not self.failure:
            p, q = self.goal.pose.position, self.goal.pose.orientation
            self.pose = [p.x, p.y, 2 * math.atan2(q.z, q.w)]
        stamp = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'map', 'base_footprint', stamp
        tf.transform.translation.x, tf.transform.translation.y = self.pose[:2]
        tf.transform.rotation.z = math.sin(self.pose[2] / 2)
        tf.transform.rotation.w = math.cos(self.pose[2] / 2)
        self.tf.sendTransform(tf)
        if self.send_odom:
            odom = Odometry()
            odom.header.frame_id, odom.child_frame_id = 'odom', 'base_footprint'
            odom.header.stamp = stamp
            odom.twist.twist.linear.x = self.actual_v
            self.odom_pub.publish(odom)
        current = stamp_ns(self.goal.header.stamp) if self.goal else 0
        code = self.lqr_error or ('GOAL_REACHED' if self.auto_reach and self.goal else 'WAIT_PATH')
        self.lqr_diag.publish(diagnostic(
            code, path_stamp_ns=current, detail='nearest_distance=0.4 limit=0.2'))
        for key, value in [('lqr', code), ('planner', 'SUCCESS'), ('safety', self.safety)]:
            self.status[key].publish(String(data=value))
        self.range_pub.publish(Float64(data=0.15 if self.safety == 'BLOCKED' else 2.0))
        self.raw_pub.publish(Twist())
        self.cmd_pub.publish(Twist())
        if self.goal and self.auto_reach and self.respond_plan and not self.failure:
            self.reached_pub.publish(self.goal)


@pytest.fixture
def system(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = str(155 + os.getpid() % 20)
    mission = dict(configured=True, frame='map',
                   home=dict(name='home', x=0.25, y=0.25, yaw=0.0),
                   pickup=dict(name='pickup', x=1.25, y=0.25, yaw=1.0),
                   dropoffs=[dict(name='far', x=3.25, y=0.25, yaw=2.0),
                             dict(name='near', x=2.25, y=0.25, yaw=-1.0)])
    points = tmp_path / 'points.yaml'
    points.write_text(yaml.safe_dump(mission))
    params = dict(use_sim_time=False, points_file=str(points),
                  log_directory=str(tmp_path / 'logs'),
                  pickup_delay_sec=0.12, dropoff_delay_sec=0.12,
                  planning_timeout_sec=0.4, navigation_timeout_sec=0.8,
                  stop_timeout_sec=0.3, preparation_timeout_sec=1.0)
    arguments = ['--ros-args']
    for key, value in params.items():
        text = str(value).lower() if isinstance(value, bool) else str(value)
        arguments.extend(['-p', f'{key}:={text}'])
    rclpy.init(args=arguments)
    task = TaskManager()
    peers = Peers()
    executor = SingleThreadedExecutor()
    executor.add_node(task)
    executor.add_node(peers)
    pump(executor, lambda: task.grid is not None and task.odom is not None and
         'lqr_diagnostic' in task.snapshot and 'safety' in task.snapshot and
         task.inhibit.service_is_ready() and peers.start.service_is_ready())
    # TF and discovery are delivered independently of other subscriptions.
    end = time.monotonic() + 0.1
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)
    yield task, peers, executor, points
    executor.shutdown()
    task.destroy_node()
    peers.destroy_node()
    rclpy.shutdown()


def start(system):
    task, peers, executor, _ = system
    future = peers.start.call_async(Trigger.Request())
    pump(executor, future.done)
    assert future.result().success, future.result().message
    return task, peers, executor


def test_round_snapshot_order_dedup_and_reload(system):
    task, peers, executor = start(system)
    _, _, _, points = system
    changed = yaml.safe_load(points.read_text())
    changed['pickup']['x'] = 1.75
    points.write_text(yaml.safe_dump(changed))
    duplicate = peers.start.call_async(Trigger.Request())
    pump(executor, duplicate.done)
    assert not duplicate.result().success
    pump(executor, lambda: task.mode == 'IDLE' and len(peers.goals) == 4)
    pump(executor, lambda: any(e['event'] == 'TASK_DONE' for e in peers.events))
    assert [g.pose.position.x for g in peers.goals] == [1.25, 2.25, 3.25, 0.25]
    assert [e['event'] for e in peers.events].count('PICKUP_DONE') == 1
    assert [e['event'] for e in peers.events].count('DROPOFF_DONE') == 2
    assert peers.inhibits == [True, False]
    start(system)
    pump(executor, lambda: len(peers.goals) == 5)
    assert peers.goals[-1].pose.position.x == 1.75


@pytest.mark.parametrize('failure,expected', [
    ('NO_PATH', 'NO_PATH'), ('TRACKING_ERROR', 'TRACKING_ERROR')])
def test_specific_failure_stops_and_logs(system, failure, expected):
    task, peers, executor, _ = system
    if failure == 'NO_PATH':
        peers.failure = failure
    else:
        peers.lqr_error = failure
    start(system)
    pump(executor, lambda: task.mode == 'FAILED')
    assert task.stop_confirmed
    assert task.failure['code'] == expected
    assert peers.inhibits[-1] is True
    records = [json.loads(line) for line in task.journal.path.read_text().splitlines()]
    record = next(r for r in records if r['event'] == 'TASK_FAILED')
    assert record['reason']['detail']
    assert 'safety' in record['inputs']
    assert not any(e['event'] == 'PICKUP_DONE' for e in peers.events)


def test_old_arrival_cannot_complete_new_goal_and_block_timeout(system):
    task, peers, executor, _ = system
    peers.auto_reach = False
    peers.safety = 'BLOCKED'
    start(system)
    pump(executor, lambda: task.active_path is not None)
    old = deepcopy(peers.goal)
    old.header.stamp.sec -= 1
    peers.reached_pub.publish(old)
    pump(executor, lambda: task.mode == 'FAILED')
    assert task.failure['code'] == 'NAVIGATION_TIMEOUT'
    assert not peers.events


def test_planning_timeout(system):
    task, peers, executor, _ = system
    peers.respond_plan = False
    start(system)
    pump(executor, lambda: task.mode == 'FAILED')
    assert task.failure['code'] == 'PLANNING_TIMEOUT'


def test_temporary_block_can_recover(system):
    task, peers, executor, _ = system
    peers.auto_reach = False
    peers.safety = 'BLOCKED'
    start(system)
    pump(executor, lambda: task.active_path is not None)
    assert task.mode == 'ACTIVE'
    peers.safety, peers.auto_reach = 'CLEAR', True
    pump(executor, lambda: task.mode == 'IDLE')


def test_log_write_failure_stops(system):
    task, peers, executor, _ = system
    peers.auto_reach = False
    start(system)
    pump(executor, lambda: task.active_path is not None)
    task.journal.path.unlink()
    task.journal.path.mkdir()
    peers.safety = 'BLOCKED'
    pump(executor, lambda: task.mode == 'FAILED')
    assert task.failure['code'] == 'LOG_WRITE_FAILED'
    assert peers.inhibits[-1] is True


def test_stop_unconfirmed_rejects_start_then_recovers(system):
    task, peers, executor, _ = system
    peers.auto_reach = False
    start(system)
    pump(executor, lambda: task.active_path is not None)
    peers.send_odom = False
    pump(executor, lambda: task.mode == 'STOPPING')
    pump(executor, lambda: getattr(task, 'stop_warning_sent', False))
    assert not task.stop_confirmed
    future = peers.start.call_async(Trigger.Request())
    pump(executor, future.done)
    assert not future.result().success
    peers.send_odom = True
    pump(executor, lambda: task.mode == 'FAILED')
    assert task.stop_confirmed


def test_invalid_reloaded_configuration_is_logged_without_motion(system):
    task, peers, executor, points = system
    data = yaml.safe_load(points.read_text())
    data['home']['x'] = float('nan')
    points.write_text(yaml.safe_dump(data))
    future = peers.start.call_async(Trigger.Request())
    pump(executor, future.done)
    assert not future.result().success
    assert 'home.x' in future.result().message
    assert not peers.goals and not peers.inhibits
    records = [json.loads(line) for line in task.journal.path.read_text().splitlines()]
    assert records[-1]['event'] == 'START_REJECTED'


def test_clock_reset_terminates_instead_of_completing_action(system):
    from rclpy.parameter import Parameter
    from rosgraph_msgs.msg import Clock
    task, peers, executor, _ = system
    peers.auto_reach = False
    start(system)
    pump(executor, lambda: task.active_path is not None)
    # Switch to a new simulation clock epoch while a goal is executing.
    task.set_parameters([Parameter('use_sim_time', value=True)])
    clock = peers.create_publisher(Clock, '/clock', 10)
    msg = Clock()
    msg.clock.sec = 10
    clock.publish(msg)
    pump(executor, lambda: task.mode == 'STOPPING')
    assert task.failure['code'] == 'CLOCK_RESET'
    assert not any(e['event'].endswith('_DONE') for e in peers.events)

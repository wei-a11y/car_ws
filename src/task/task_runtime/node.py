# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""ROS boundary for missions: freshness, correlation, inhibition and durable logs."""
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import signal
import time
import uuid

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from task_runtime.core import MissionEngine, load_mission
from task_runtime.journal import Journal, json_safe


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def yaw_of(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values) or abs(math.hypot(*values) - 1.0) > 1e-6:
        raise ValueError('Invalid quaternion')
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle_error(a, b):
    return math.remainder(a - b, 2 * math.pi)


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        descriptor = ParameterDescriptor(read_only=True)

        def parameter(name, default):
            return self.declare_parameter(name, default, descriptor).value

        self.p = {}
        defaults = dict(
            planning_frame='map', base_frame='base_footprint', odom_frame='odom',
            points_file='', log_directory='src/task/logs',
            pickup_delay_sec=3.0, dropoff_delay_sec=3.0, planning_timeout_sec=10.0,
            navigation_timeout_sec=300.0, preparation_timeout_sec=10.0, stop_timeout_sec=5.0,
            input_timeout_sec=1.0, tf_max_age_sec=0.5, odom_max_age_sec=0.25,
            position_tolerance=0.03, heading_tolerance=0.05,
            stopped_linear_tolerance=0.01, stopped_angular_tolerance=0.02,
            tick_period_sec=0.05, log_max_bytes=10485760, log_backups=5)
        for name, default in defaults.items():
            self.p[name] = parameter(name, default)
            if isinstance(default, (float, int)):
                if not math.isfinite(self.p[name]) or self.p[name] <= 0:
                    raise ValueError(f'{name} must be finite and positive')
        self.frame = self.p['planning_frame']
        self.base = self.p['base_frame']
        if not self.frame or not self.base or self.frame == self.base:
            raise ValueError('planning_frame/base_frame must be distinct, nonempty frames')
        topics = dict(
            goal_topic='/goal_pose', path_topic='/plan', grid_topic='/plan/inflated_grid',
            odom_topic='/odom', planner_status_topic='/plan/status',
            planner_diagnostics_topic='/plan/diagnostics',
            lqr_status_topic='/controller/lqr/status',
            lqr_diagnostics_topic='/controller/lqr/diagnostics',
            reached_goal_topic='/controller/lqr/reached_goal',
            inhibit_service='/controller/lqr/set_inhibit',
            safety_status_topic='/controller/safety/status',
            safety_range_topic='/controller/safety/min_range', raw_cmd_topic='/cmd_vel_raw',
            cmd_vel_topic='/cmd_vel', start_service='/task/start',
            status_topic='/task/status', events_topic='/task/events')
        self.topics = {name: parameter(name, default) for name, default in topics.items()}
        if any(not value for value in self.topics.values()):
            raise ValueError('Topic and service names must not be empty')
        self.engine = MissionEngine(self.p['pickup_delay_sec'], self.p['dropoff_delay_sec'])
        self.mode = 'IDLE'
        self.task_id = None
        self.task_started = self.stage_started = time.monotonic()
        self.phase_started = self.stage_started
        self.last_phase = None
        self.last_status = None
        self.journal = None
        self.log_broken = False
        self.snapshot = {}
        self.grid = None
        self.odom = None
        self.odom_received = 0.0
        self.goal_stamp = None
        self.active_path = None
        self.planner_success = False
        self.future = None
        self.ack_time = None
        self.stop_confirmed = True
        self.failure = None
        self.last_ros = None
        self.last_clock_progress = time.monotonic()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, self.topics['goal_topic'], 1)
        self.status_pub = self.create_publisher(String, self.topics['status_topic'], latched)
        self.events_pub = self.create_publisher(String, self.topics['events_topic'], 20)
        self.inhibit = self.create_client(SetBool, self.topics['inhibit_service'])
        self.create_service(Trigger, self.topics['start_service'], self.on_start)
        self.create_subscription(OccupancyGrid, self.topics['grid_topic'], self.on_grid, latched)
        self.create_subscription(Odometry, self.topics['odom_topic'], self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Path, self.topics['path_topic'], self.on_path, 10)
        self.create_subscription(
            PoseStamped, self.topics['reached_goal_topic'], self.on_reached, 10)
        for key in ('planner', 'lqr', 'safety'):
            self.create_subscription(String, self.topics[key + '_status_topic'],
                                     lambda msg, key=key: self.remember(key, msg.data), latched)
        self.create_subscription(DiagnosticArray, self.topics['planner_diagnostics_topic'],
                                 lambda msg: self.on_diagnostics('planner', msg), 10)
        self.create_subscription(DiagnosticArray, self.topics['lqr_diagnostics_topic'],
                                 lambda msg: self.on_diagnostics('lqr', msg), 10)
        self.create_subscription(Float64, self.topics['safety_range_topic'],
                                 lambda msg: self.remember('min_range', msg.data), 10)
        for key in ('raw_cmd', 'cmd_vel'):
            self.create_subscription(Twist, self.topics[key + '_topic'],
                                     lambda msg, key=key: self.remember(
                                         key, {'v': msg.linear.x, 'w': msg.angular.z}), 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(self.p['tick_period_sec'], self.tick, clock=self.wall_clock)
        self.publish_status()

    @property
    def phase(self):
        return self.engine.state if self.mode == 'ACTIVE' else self.mode

    def remember(self, key, value):
        old = self.snapshot.get(key)
        self.snapshot[key] = {'value': value, 'received': time.monotonic()}
        if key in ('planner', 'lqr', 'safety') and self.mode == 'ACTIVE':
            if old is None or old['value'] != value:
                self.record('INPUT_STATUS', source=key, value=value)

    def failure_context(self):
        now = time.monotonic()
        keys = ('planner', 'lqr', 'safety', 'min_range', 'raw_cmd', 'cmd_vel',
                'planner_diagnostic', 'lqr_diagnostic')
        return {key: ({'value': self.snapshot[key]['value'],
                       'age_sec': now - self.snapshot[key]['received']}
                      if key in self.snapshot else 'unavailable') for key in keys}

    def record(self, event, **extra):
        record = dict(wall_time=datetime.now(timezone.utc).isoformat(),
                      ros_time_ns=self.get_clock().now().nanoseconds,
                      task_id=self.task_id, phase=extra.pop('phase', self.phase), event=event,
                      elapsed_sec=time.monotonic() - self.task_started,
                      phase_elapsed_sec=time.monotonic() - self.phase_started,
                      target=extra.pop(
                          'target', asdict(self.engine.target) if self.engine.target else None),
                      **extra)
        text = json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False)
        if event in ('TASK_FAILED', 'STOP_UNCONFIRMED'):
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)
        if self.journal is None:
            return False
        try:
            self.journal.write(record)
            return True
        except (OSError, ValueError) as error:
            self.log_broken = True
            self.journal = None
            self.get_logger().error(f'LOG_WRITE_FAILED: {error}; terminating current task')
            return False

    def publish_status(self):
        if self.phase != self.last_phase:
            self.phase_started = time.monotonic()
            self.last_phase = self.phase
        data = json.dumps(dict(task_id=self.task_id, state=self.phase,
                               point=self.engine.target.name if self.engine.target else None,
                               failure=self.failure, stop_confirmed=self.stop_confirmed),
                          ensure_ascii=False)
        if data != self.last_status:
            self.status_pub.publish(String(data=data))
            if self.journal:
                self.record('STATE_CHANGED', status=json.loads(data))
            self.last_status = data

    def on_grid(self, msg):
        self.grid = msg
        if self.mode == 'ACTIVE':
            self.fail('MAP_CHANGED', 'Inflated grid changed; current navigation is invalidated')

    def on_odom(self, msg):
        self.odom, self.odom_received = msg, time.monotonic()

    def fresh(self, stamp, limit):
        age = (self.get_clock().now().nanoseconds - stamp_ns(stamp)) / 1e9
        return stamp.sec >= 0 and stamp.nanosec < 1000000000 and 0 <= age <= limit

    def current_pose(self):
        transform = self.buffer.lookup_transform(self.frame, self.base, Time())
        if not self.fresh(transform.header.stamp, self.p['tf_max_age_sec']):
            raise ValueError('TF_UNAVAILABLE: map/base transform is stale or future-dated')
        p = transform.transform.translation
        if not all(math.isfinite(x) for x in (p.x, p.y, p.z)):
            raise ValueError('TF_UNAVAILABLE: nonfinite translation')
        return p.x, p.y, yaw_of(transform.transform.rotation)

    def odom_issue(self):
        if self.odom is None:
            return 'No odometry received'
        msg = self.odom
        actual_frames = f'{msg.header.frame_id}->{msg.child_frame_id}'
        expected_frames = f'{self.p["odom_frame"]}->{self.base}'
        if actual_frames != expected_frames:
            return f'frames={actual_frames}, expected={expected_frames}'
        age = (self.get_clock().now().nanoseconds - stamp_ns(msg.header.stamp)) / 1e9
        wall_age = time.monotonic() - self.odom_received
        if not self.fresh(msg.header.stamp, self.p['odom_max_age_sec']):
            return f'signed_age={age:.6f}s, limit={self.p["odom_max_age_sec"]}s'
        if wall_age > self.p['input_timeout_sec']:
            return f'receipt_age={wall_age:.6f}s, limit={self.p["input_timeout_sec"]}s'
        v = msg.twist.twist
        velocity = (v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)
        if not all(math.isfinite(x) for x in velocity):
            return f'Nonfinite measured Twist: {velocity}'
        return None

    def odom_valid(self):
        return self.odom_issue() is None

    def stopped(self):
        if not self.odom_valid():
            return False
        v = self.odom.twist.twist
        return (math.hypot(v.linear.x, v.linear.y, v.linear.z) <=
                self.p['stopped_linear_tolerance'] and
                math.hypot(v.angular.x, v.angular.y, v.angular.z) <=
                self.p['stopped_angular_tolerance'])

    def grid_endpoint(self, point):
        g = self.grid
        if (g is None or g.header.frame_id != self.frame or
                g.info.width <= 0 or g.info.height <= 0 or
                len(g.data) != g.info.width * g.info.height or
                not math.isfinite(g.info.resolution) or g.info.resolution <= 0):
            raise ValueError('MAP_UNAVAILABLE: no valid inflated grid in planning frame')
        origin = g.info.origin
        if (not all(math.isfinite(v) for v in (origin.position.x, origin.position.y,
                                               origin.position.z)) or
                abs(origin.orientation.x) > 1e-6 or abs(origin.orientation.y) > 1e-6):
            raise ValueError('MAP_UNAVAILABLE: invalid/nonplanar grid origin')
        yaw = yaw_of(origin.orientation)
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = point.x - origin.position.x, point.y - origin.position.y
        x = math.floor((c * dx + s * dy) / g.info.resolution)
        y = math.floor((-s * dx + c * dy) / g.info.resolution)
        if not (0 <= x < g.info.width and 0 <= y < g.info.height):
            raise ValueError(f'INVALID_GOAL: {point.name} is outside inflated grid')
        if g.data[y * g.info.width + x] != 0:
            raise ValueError(f'INVALID_GOAL: {point.name} is occupied/unknown in inflated grid')
        cx, cy = (x + 0.5) * g.info.resolution, (y + 0.5) * g.info.resolution
        return origin.position.x + c * cx - s * cy, origin.position.y + s * cx + c * cy

    def ready(self):
        if self.get_clock().now().nanoseconds <= 0:
            raise ValueError('CLOCK_UNAVAILABLE: no positive ROS time')
        self.current_pose()
        odom_issue = self.odom_issue()
        if odom_issue:
            raise ValueError(f'ODOM_UNAVAILABLE: {odom_issue}')
        for key in ('safety', 'min_range', 'lqr_diagnostic'):
            item = self.snapshot.get(key)
            if item is None or time.monotonic() - item['received'] > self.p['input_timeout_sec']:
                raise ValueError(f'INPUT_UNAVAILABLE: {key} missing or stale')
        if not self.inhibit.service_is_ready() or self.goal_pub.get_subscription_count() == 0:
            raise ValueError(
                'NAVIGATION_UNAVAILABLE: planner subscription or inhibit service missing')

    def on_start(self, request, response):
        del request
        if self.mode not in ('IDLE', 'FAILED') or not self.stop_confirmed:
            response.message = (f'START_REJECTED: state={self.phase}, '
                                f'stop_confirmed={self.stop_confirmed}')
            self.get_logger().warning(response.message)
            return response
        try:
            self.journal = Journal(
                self.p['log_directory'], self.p['log_max_bytes'], self.p['log_backups'])
            mission = load_mission(self.p['points_file'], self.frame)
            self.ready()
            for point in (mission.home, mission.pickup, *mission.dropoffs):
                self.grid_endpoint(point)
        except (OSError, ValueError, TransformException) as error:
            response.message = f'START_REJECTED: {error}'
            self.get_logger().error(response.message)
            if self.journal:
                self.record('START_REJECTED', reason=str(error))
            return response
        self.task_id = str(uuid.uuid4())
        self.task_started = self.stage_started = time.monotonic()
        self.pending_mission = mission
        self.log_broken = False
        self.failure = None
        if not self.record('CONFIG_LOCKED', mission=asdict(mission)):
            response.message = 'START_REJECTED: cannot write task journal'
            return response
        self.mode = 'PREPARING'
        self.stop_confirmed = False
        self.future = self.inhibit.call_async(SetBool.Request(data=True))
        self.ack_time = None
        self.stop_warning_sent = False
        response.success = True
        response.message = f'Accepted task {self.task_id}; preparing, not yet moving'
        self.publish_status()
        return response

    def fail(self, code, detail):
        if self.mode in ('STOPPING', 'FAILED'):
            return
        self.failure = {'code': code, 'detail': detail}
        self.record('TASK_FAILED', reason=self.failure, inputs=self.failure_context())
        self.engine.fail()
        self.mode = 'STOPPING'
        self.stop_confirmed = False
        self.stage_started = time.monotonic()
        self.ack_time = None
        self.stop_warning_sent = False
        self.future = self.inhibit.call_async(SetBool.Request(data=True))
        self.publish_status()

    def on_diagnostics(self, source, msg):
        for status in msg.status:
            values = {item.key: item.value for item in status.values}
            self.remember(source + '_diagnostic', dict(state=status.message, **values))
            if self.goal_stamp is None:
                continue
            key = 'goal_stamp_ns' if source == 'planner' else 'path_stamp_ns'
            if values.get(key) != str(self.goal_stamp):
                continue
            if (self.mode in ('STOPPING', 'FAILED') and source == 'planner' and
                    self.failure and self.failure['code'] == 'PATH_CLEARED' and
                    status.message != 'SUCCESS'):
                # Independent DDS topics can deliver the empty Path before its reason.
                self.failure = {'code': status.message, 'detail': values.get('detail', '')}
                self.record('FAILURE_DETAIL', reason=self.failure, inputs=self.failure_context())
            if self.mode != 'ACTIVE':
                continue
            if source == 'planner':
                if status.message != 'SUCCESS':
                    self.fail(status.message, values.get(
                        'detail', 'Planner failure; detail unavailable'))
                else:
                    self.planner_success = values.get('path_stamp_ns') == str(self.goal_stamp)
            elif status.message in ('TRACKING_ERROR', 'PATH_INVALID', 'PATH_UNAVAILABLE',
                                    'TF_UNAVAILABLE', 'ODOM_UNAVAILABLE', 'CLOCK_UNAVAILABLE'):
                self.fail(status.message, values.get('detail', 'Controller detail unavailable'))

    def on_path(self, msg):
        if self.mode != 'ACTIVE' or self.goal_stamp is None:
            return
        stamp = stamp_ns(msg.header.stamp)
        if not msg.poses and stamp >= self.goal_stamp:
            # Planner emits a detailed failure immediately after clearing the path.
            # Defer generic failure until the next wall tick so that detail wins.
            self.path_cleared = True
            return
        if stamp != self.goal_stamp:
            return
        try:
            if msg.header.frame_id != self.frame or not msg.poses:
                raise ValueError('Wrong path frame or empty path')
            end = msg.poses[-1].pose
            expected_x, expected_y = self.grid_endpoint(self.engine.target)
            if (math.hypot(end.position.x - expected_x, end.position.y - expected_y) > 1e-6 or
                    abs(angle_error(yaw_of(end.orientation), self.engine.target.yaw)) > 1e-6):
                raise ValueError('Path terminal position/orientation does not match current goal')
            self.active_path = msg
        except (ValueError, OverflowError) as error:
            self.fail('PATH_MISMATCH', str(error))

    def at_endpoint(self):
        if not self.active_path or not self.stopped():
            return False
        x, y, yaw = self.current_pose()
        target = self.active_path.poses[-1].pose
        return (math.hypot(x - target.position.x, y - target.position.y) <=
                self.p['position_tolerance'] and
                abs(angle_error(yaw, yaw_of(target.orientation))) <= self.p['heading_tolerance'])

    def on_reached(self, msg):
        if (self.mode != 'ACTIVE' or not self.engine.state.startswith('NAV_') or
                not self.active_path or not self.planner_success or
                stamp_ns(msg.header.stamp) != self.goal_stamp or
                msg.header.frame_id != self.frame or
                msg.pose != self.active_path.poses[-1].pose):
            return
        try:
            if self.at_endpoint():
                self.effects(self.engine.reached(self.get_clock().now().nanoseconds / 1e9))
        except (ValueError, TransformException) as error:
            self.fail('ARRIVAL_INPUT_INVALID', str(error))

    def effects(self, effects):
        for event, value in effects:
            payload = ([asdict(p) for p in value] if event == 'DELIVERY_ORDER' else asdict(value))
            event_target = self.engine.mission.pickup if event == 'DELIVERY_ORDER' else value
            event_phase = {'PICKUP_DONE': 'PICKING', 'DROPOFF_DONE': 'DROPPING',
                           'DELIVERY_ORDER': 'PICKING', 'TASK_DONE': 'NAV_HOME'}.get(
                               event, self.phase)
            if not self.record(
                    event, payload=payload, target=asdict(event_target), phase=event_phase):
                self.fail('LOG_WRITE_FAILED', 'Cannot persist task event')
                return
            if event == 'GOAL':
                self.stop_confirmed = False
                msg = PoseStamped()
                msg.header.frame_id = self.frame
                msg.header.stamp = self.get_clock().now().to_msg()
                self.goal_stamp = stamp_ns(msg.header.stamp)
                msg.pose.position.x, msg.pose.position.y = value.x, value.y
                msg.pose.orientation.z = math.sin(value.yaw / 2)
                msg.pose.orientation.w = math.cos(value.yaw / 2)
                self.active_path, self.planner_success, self.path_cleared = None, False, False
                self.stage_started = time.monotonic()
                self.goal_pub.publish(msg)
            else:
                self.events_pub.publish(String(data=json.dumps(json_safe(dict(
                    task_id=self.task_id, event=event, payload=payload)), ensure_ascii=False)))
            if event == 'TASK_DONE':
                self.mode, self.stop_confirmed = 'IDLE', True
        self.publish_status()

    def service_ack(self):
        if not self.future or not self.future.done():
            return False
        try:
            result = self.future.result()
            if result is None or not result.success:
                raise ValueError('Controller rejected inhibition change')
        except Exception as error:
            if self.mode != 'STOPPING':
                self.fail('INHIBIT_SERVICE_FAILED', str(error))
            return False
        if self.ack_time is None:
            self.ack_time = time.monotonic()
            self.ack_ros = self.get_clock().now().nanoseconds
        return True

    def tick(self):
        try:
            self.tick_checked()
        except (ValueError, OSError, TransformException, OverflowError) as error:
            if self.mode in ('ACTIVE', 'PREPARING', 'RELEASING'):
                prefix = str(error).split(':', 1)[0]
                codes = {'ODOM_UNAVAILABLE', 'TF_UNAVAILABLE', 'CLOCK_UNAVAILABLE',
                         'INPUT_UNAVAILABLE', 'NAVIGATION_UNAVAILABLE', 'MAP_UNAVAILABLE'}
                code = 'TF_UNAVAILABLE' if isinstance(error, TransformException) else (
                    prefix if prefix in codes else 'INPUT_INVALID')
                self.fail(code, str(error))
            else:
                self.get_logger().error(f'Task input error: {error}')
        self.publish_status()

    def tick_checked(self):
        wall = time.monotonic()
        ros = self.get_clock().now().nanoseconds
        if self.mode in ('PREPARING', 'RELEASING', 'ACTIVE'):
            if self.log_broken:
                self.fail('LOG_WRITE_FAILED', 'Task journal unavailable')
                return
            if self.last_ros is not None and ros < self.last_ros:
                self.fail('CLOCK_RESET', 'ROS time moved backwards; task cannot resume')
                self.last_ros = ros
                return
            if ros != self.last_ros:
                self.last_clock_progress = wall
            elif wall - self.last_clock_progress > self.p['input_timeout_sec']:
                self.fail('CLOCK_UNAVAILABLE', 'ROS clock stopped; delays cannot complete')
                return
        else:
            self.last_clock_progress = wall
        self.last_ros = ros
        if self.mode in ('PREPARING', 'STOPPING'):
            ack = self.service_ack()
            if ack and self.odom_received >= self.ack_time and self.stopped():
                self.stop_confirmed = True
                if self.mode == 'STOPPING':
                    self.record('STOP_CONFIRMED', reason=self.failure)
                    self.mode = 'FAILED'
                else:
                    self.ready()
                    self.mode = 'RELEASING'
                    self.future = self.inhibit.call_async(SetBool.Request(data=False))
                    self.ack_time = None
                return
            limit_key = ('stop_timeout_sec' if self.mode == 'STOPPING'
                         else 'preparation_timeout_sec')
            limit = self.p[limit_key]
            if wall - self.stage_started > limit:
                if self.mode == 'STOPPING':
                    # Continue seeking confirmation; a later reply can resolve this state.
                    if not getattr(self, 'stop_warning_sent', False):
                        self.record(
                            'STOP_UNCONFIRMED', reason=self.failure,
                            detail='Inhibit response and fresh stopped odometry not confirmed',
                            inputs=self.failure_context())
                        self.stop_warning_sent = True
                else:
                    self.fail(
                        'PREPARATION_TIMEOUT', 'Could not inhibit and confirm stationary base')
            return
        if self.mode == 'RELEASING':
            if self.service_ack() and self.mode == 'RELEASING':
                if ros <= self.ack_ros:
                    return  # Next goal must be strictly newer than controller release.
                self.ready()
                self.mode = 'ACTIVE'
                self.stop_warning_sent = False
                self.effects(self.engine.start(self.pending_mission, self.task_id))
            elif wall - self.stage_started > self.p['preparation_timeout_sec']:
                self.fail('RELEASE_TIMEOUT', 'Controller release not acknowledged')
            return
        if self.mode != 'ACTIVE':
            return
        self.ready()
        if self.path_cleared:
            self.fail(
                'PATH_CLEARED', 'Current path cleared; see planner diagnostics and map state')
            return
        if self.engine.state.startswith('NAV_'):
            elapsed = wall - self.stage_started
            awaiting_plan = self.active_path is None or not self.planner_success
            if awaiting_plan and elapsed > self.p['planning_timeout_sec']:
                self.fail(
                    'PLANNING_TIMEOUT', 'No correlated successful path before planning deadline')
            elif elapsed > self.p['navigation_timeout_sec']:
                self.fail('NAVIGATION_TIMEOUT', 'Goal not reached/aligned/stopped before deadline')
        elif self.engine.state in ('PICKING', 'DROPPING'):
            if not self.at_endpoint():
                self.fail(
                    'ACTION_POSE_LOST', 'Pose/stop confirmation lost during simulated arm action')
                return
            self.effects(self.engine.tick(ros / 1e9))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = TaskManager()
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            if node.mode in ('ACTIVE', 'PREPARING', 'RELEASING', 'STOPPING'):
                node.fail('SHUTDOWN', 'Task node shutting down; requesting motion inhibition')
                deadline = time.monotonic() + node.p['stop_timeout_sec']
                while time.monotonic() < deadline and not node.stop_confirmed:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if not node.stop_confirmed:
                    node.get_logger().error('STOP_UNCONFIRMED: shutdown stop was not confirmed')
            node.destroy_node()
            rclpy.shutdown()

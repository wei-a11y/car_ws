# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Segment admission and recoverable navigation, using the existing mission owner.

The only autonomous velocity publisher remains LQR, followed by Safety Gate.
Candidate plans never reach LQR until a fresh observation admits a finite segment.
"""
from copy import deepcopy
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import math
import os
from pathlib import Path as FilePath
import signal
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from task_runtime.core import Waypoint
from task_runtime.gazebo_backend import GazeboBackend
from task_runtime.node import TaskManager, angle_error, stamp_ns, yaw_of
from task_runtime.recovery_core import (
    GridModel, Obstacle, Pose3, RecoveryPlanner, ScanView, split_path,
)


NAV_DEFAULTS = dict(
    scan_topic='/scan', static_map_topic='/map', working_map_topic='/task/working_map',
    execution_path_topic='/plan', scan_timeout_sec=0.5, scan_wall_timeout_sec=0.6,
    corridor_spacing=0.1, stop_margin=0.3, observation_chunk_length=1.0,
    min_advance=0.1, map_settle_sec=0.1, inspection_timeout_sec=3.0,
    operation_timeout_sec=30.0, max_grasp_attempts=3, max_recovery_cycles=8,
    anonymous_obstacle_size=0.15)


def task_source_root():
    for parent in FilePath(__file__).resolve().parents:
        if (parent / 'task_runtime/core.py').is_file() and (parent / 'CMakeLists.txt').is_file():
            return parent
    raise ValueError('Recovery runtime must be installed within this workspace src/task')


def pose_message(pose, frame, stamp):
    result = PoseStamped()
    result.header.frame_id, result.header.stamp = frame, stamp
    result.pose.position.x, result.pose.position.y, result.pose.position.z = (
        float(pose.x), float(pose.y), float(pose.z))
    result.pose.orientation.z = math.sin(pose.yaw / 2)
    result.pose.orientation.w = math.cos(pose.yaw / 2)
    return result


def pose_distance(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


class RestorationLedger:
    """Intent is durable BEFORE an object can move. Never cleared on process restart."""

    def __init__(self, directory):
        self.filename = FilePath(directory) / 'pending_restoration.json'
        self.items = {}
        if self.filename.exists():
            self.items = json.loads(self.filename.read_text(encoding='utf-8'))
            if not isinstance(self.items, dict):
                raise ValueError('Invalid restoration ledger; manual inspection required')
            for item in self.items.values():
                pose = Pose3(**item['original'])
                if not all(math.isfinite(v) for v in asdict(pose).values()):
                    raise ValueError('Invalid original pose in restoration ledger')

    def save(self):
        self.filename.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.filename.with_suffix('.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(self.items, stream, allow_nan=False, ensure_ascii=False)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.filename)

    def add(self, obstacle, task_id):
        if obstacle.object_id not in self.items:
            self.items[obstacle.object_id] = {
                'original': asdict(obstacle.pose), 'task_id': task_id,
                'frame': obstacle.frame, 'label': obstacle.label}
            self.save()

    def remove(self, object_id):
        if object_id in self.items:
            previous = self.items.pop(object_id)
            try:
                self.save()
            except OSError:
                self.items[object_id] = previous
                raise


class RecoveryTaskManager(TaskManager):
    """Opt-in wrapper; the original task executable and MissionEngine stay unchanged."""

    def __init__(self, backend_factory=GazeboBackend):
        self.nav_state = 'IDLE'
        self.segment_index = 0
        self.backend = None
        self.chosen = None
        self.ledger = None
        self.execution_stamp = None
        self.static_map = None
        self.grid_model = None
        self.grid_generation = None
        self.config = {}
        self.known = {}
        self.nav_target = None
        self.nav_purpose = 'MISSION'
        self.scan = None
        self.scan_received = 0.0
        self.scan_view = None
        self.release_ros = 0
        self.last_request_stamp = 0
        self.motion_inhibited = False
        self.candidate_path = None
        self.segments = []
        self.route_cursor = None
        self.pending_failure = None
        self.excluded = set()
        self.recovery_cycles = 0
        self.grasp_attempts = 0
        self.restoration_counts = {}
        self.restoration_stamps = {}
        self.minimum_scan_stamp = 0
        self.evaluation_future = None
        self.evaluator = ThreadPoolExecutor(max_workers=1, thread_name_prefix='recovery-evaluator')
        super().__init__()
        readonly = ParameterDescriptor(read_only=True)
        root = task_source_root()
        recovery_file = self.declare_parameter(
            'recovery_file', str(root / 'config/recovery.yaml'), readonly).value
        scenarios_file = self.declare_parameter(
            'scenarios_file', str(root / 'config/recovery_scenarios.yaml'), readonly).value
        self.config = yaml.safe_load(FilePath(recovery_file).read_text(encoding='utf-8'))
        if not isinstance(self.config, dict):
            raise ValueError('Recovery YAML must be a mapping')
        geometry_config = self.config.setdefault('geometry', {})
        geometry_config['base_position_tolerance'] = max(
            geometry_config.get('base_position_tolerance', 0.03), self.p['position_tolerance'])
        # Validate costs and manipulation limits before admitting any mission.
        RecoveryPlanner(self.config, None)
        self.n = {**NAV_DEFAULTS, **self.config.get('navigation', {})}
        for key, value in NAV_DEFAULTS.items():
            if isinstance(value, (float, int)):
                actual = self.n[key]
                if isinstance(actual, bool) or not math.isfinite(actual) or actual <= 0:
                    raise ValueError(f'navigation.{key} must be finite and positive')
                if isinstance(value, int) and int(actual) != actual:
                    raise ValueError(f'navigation.{key} must be an integer')
        if self.topics['path_topic'] == self.n['execution_path_topic']:
            raise ValueError('Candidate and execution path topics must be distinct')
        log_dir = FilePath(self.p['log_directory']).resolve()
        if not log_dir.is_relative_to(root):
            raise ValueError('Recovery log_directory must be inside src/task')
        self.ledger = RestorationLedger(log_dir)
        backend_config = dict(self.config.get('backend', {}))
        backend_config.update(backend_registry_file=scenarios_file, planning_frame=self.frame)
        geometry = self.config.get('geometry', {})
        for key in ('arm_reach', 'lift_height'):
            if key in geometry:
                backend_config[key] = geometry[key]
        self.backend = backend_factory(self, backend_config, self.buffer)
        self.backend_config = backend_config
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.working_pub = self.create_publisher(
            OccupancyGrid, self.n['working_map_topic'], latched)
        self.execution_pub = self.create_publisher(Path, self.n['execution_path_topic'], 1)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.n.get('marker_topic', '/task/recovery/markers'), 1)
        self.create_subscription(OccupancyGrid, self.n['static_map_topic'],
                                 self.on_static_map, latched)
        self.create_subscription(LaserScan, self.n['scan_topic'], self.on_scan,
                                 qos_profile_sensor_data)
        self.state_started = time.monotonic()
        self.publish_status()

    @property
    def phase(self):
        if self.mode == 'ACTIVE' and self.engine.state.startswith('NAV_'):
            return self.nav_state
        return super().phase

    def publish_status(self):
        if self.phase != self.last_phase:
            self.phase_started = time.monotonic()
            self.last_phase = self.phase
        data = dict(task_id=self.task_id, state=self.phase, mission_state=self.engine.state,
                    point=self.engine.target.name if self.engine.target else None,
                    failure=self.failure, stop_confirmed=self.stop_confirmed,
                    segment_index=self.segment_index,
                    object_id=self.chosen.object_id if self.chosen else None,
                    scheme=self.chosen.kind if self.chosen else None,
                    grasp_attempts=self.grasp_attempts,
                    pending_restoration=list(self.ledger.items) if self.ledger else [])
        if self.backend:
            data['operation'] = self.backend.operation
            data['held_id'] = self.backend.held_id
        text = json.dumps(data, ensure_ascii=False, allow_nan=False)
        if text != self.last_status:
            self.status_pub.publish(String(data=text))
            if self.journal:
                self.record('STATE_CHANGED', status=data)
            self.last_status = text

    def event(self, event, **values):
        if not self.record(event, **values):
            self.fail('LOG_WRITE_FAILED', f'Cannot persist {event}')
            return False
        self.events_pub.publish(String(data=json.dumps(
            dict(task_id=self.task_id, event=event, **values), allow_nan=False)))
        return True

    def transition(self, state):
        self.nav_state = state
        self.state_started = time.monotonic()

    def on_static_map(self, msg):
        if self.static_map and (msg.info == self.static_map.info and
                                msg.header.frame_id == self.static_map.header.frame_id and
                                msg.data == self.static_map.data):
            return
        if self.mode not in ('IDLE', 'FAILED'):
            self.fail('MAP_CHANGED', 'Static localization map changed during the mission')
            return
        if msg.header.frame_id != self.frame:
            self.get_logger().error('Static map is not in planning_frame')
            return
        try:
            self.grid_model = GridModel.from_message(msg, self.config.get('planning', {}))
            self.static_map = deepcopy(msg)
            self.publish_working_map()
        except (ValueError, RuntimeError) as error:
            self.get_logger().error(f'MAP_UNAVAILABLE: {error}')

    def on_grid(self, msg):
        if self.grid_generation is None:
            return
        if stamp_ns(msg.header.stamp) != self.grid_generation:
            if self.mode == 'ACTIVE' and stamp_ns(msg.header.stamp) > self.grid_generation:
                self.fail('UNEXPECTED_MAP_UPDATE', 'Inflated grid does not match task snapshot')
            return
        if (msg.header.frame_id != self.frame or msg.info != self.static_map.info or
                len(msg.data) != len(self.static_map.data)):
            self.fail('MAP_MISMATCH', 'Inflated snapshot geometry changed')
            return
        self.grid = msg
        self.grid_ack_time = time.monotonic()

    def on_scan(self, msg):
        self.scan, self.scan_received = msg, time.monotonic()

    def base_pose(self):
        x, y, yaw = self.current_pose()
        return Pose3(x, y, 0.0, yaw)

    def refresh_observation(self):
        if (self.scan is None or
                time.monotonic() - self.scan_received > self.n['scan_wall_timeout_sec'] or
                not self.fresh(self.scan.header.stamp, self.n['scan_timeout_sec'])):
            raise ValueError('SCAN_UNAVAILABLE: missing, stale or future scan')
        transform = self.buffer.lookup_transform(
            self.frame, self.scan.header.frame_id, Time.from_msg(self.scan.header.stamp))
        t = transform.transform.translation
        self.scan_view = ScanView(
            self.scan, Pose3(t.x, t.y, t.z, yaw_of(transform.transform.rotation)),
            allow_positive_infinity=self.n.get('allow_positive_infinity', False))
        for obstacle in self.backend.observations(self.scan_view.visible):
            self.known[obstacle.object_id] = obstacle

    def ready(self):
        super().ready()
        if self.backend is None:
            raise ValueError('BACKEND_UNAVAILABLE: not initialized')
        ok, reason = self.backend.ready()
        if not ok:
            raise ValueError(f'BACKEND_UNAVAILABLE: {reason}')
        if self.static_map is None or self.grid_model is None:
            raise ValueError('MAP_UNAVAILABLE: no static map')
        self.refresh_observation()
        if self.backend.held_id and self.backend.operation == 'IDLE':
            observation = self.backend.get_observation(self.backend.held_id)
            if observation is None:
                raise ValueError('HELD_OBJECT_UNAVAILABLE: no fresh attachment observation')
            tcp = self.backend.tcp
            if tcp and math.sqrt((observation.pose.x-tcp.x)**2 +
                                 (observation.pose.y-tcp.y)**2 +
                                 (observation.pose.z-tcp.z)**2) > self.backend_config.get(
                                     'follow_tolerance', 0.06):
                raise ValueError('HELD_OBJECT_UNAVAILABLE: attachment stopped following base')
        if self.execution_pub.get_subscription_count() == 0:
            raise ValueError('NAVIGATION_UNAVAILABLE: LQR execution path subscriber missing')

    def verify_ledger(self):
        """Manual restoration may clear an old intent only through fresh measured poses."""
        if self.backend.held_id:
            return
        required = self.backend_config.get('verification_samples', 3)
        for object_id, item in list(self.ledger.items.items()):
            observation = self.backend.get_observation(object_id)
            if observation is None or observation.stamp_ns == self.restoration_stamps.get(object_id):
                continue
            self.restoration_stamps[object_id] = observation.stamp_ns
            target = Pose3(**item['original'])
            d = math.sqrt((observation.pose.x-target.x)**2 +
                          (observation.pose.y-target.y)**2 + (observation.pose.z-target.z)**2)
            aligned = (d <= self.backend_config.get('restore_position_tolerance', 0.04) and
                       abs(angle_error(observation.pose.yaw, target.yaw)) <=
                       self.backend_config.get('restore_yaw_tolerance', 0.08))
            self.restoration_counts[object_id] = (
                self.restoration_counts.get(object_id, 0) + 1 if aligned else 0)
            if self.restoration_counts[object_id] >= required:
                self.ledger.remove(object_id)

    def on_start(self, request, response):
        if self.ledger and self.ledger.items:
            response.message = 'START_REJECTED: unresolved restoration; restore registered objects first'
            return response
        result = super().on_start(request, response)
        if result.success:
            self.excluded.clear()
            self.known.clear()
            self.chosen = None
            self.pending_failure = None
            self.recovery_cycles = 0
            self.grasp_attempts = 0
            self.record('RECOVERY_CONFIG_LOCKED', recovery=self.config)
        return result

    def effects(self, effects):
        for event, value in effects:
            if event != 'GOAL':
                super().effects([(event, value)])
                continue
            if not self.event('GOAL', payload=asdict(value)):
                return
            self.mission_started = time.monotonic()
            self.recovery_cycles = 0
            self.chosen = None
            self.plan_to(Pose3(value.x, value.y, 0.0, value.yaw), 'MISSION')

    def stop_then(self, continuation):
        self.execution_stamp = None
        self.goal_stamp = None
        self.stop_confirmed = False
        self.nav_future = self.inhibit.call_async(SetBool.Request(data=True))
        self.nav_ack_time = None
        self.stop_continuation = continuation
        self.transition('NAV_STOPPING')

    def plan_to(self, target, purpose):
        self.nav_target, self.nav_purpose = target, purpose
        self.stop_then(self.prepare_plan)

    def scan_hits(self):
        # World-space endpoints only. No freespace ray clearing of the static map.
        scan, origin = self.scan, self.scan_view.origin
        hits = []
        for i, value in enumerate(scan.ranges):
            if not math.isfinite(value) or not (scan.range_min <= value < scan.range_max):
                continue
            angle = origin.yaw + scan.angle_min + i * scan.angle_increment
            point = (origin.x + value * math.cos(angle), origin.y + value * math.sin(angle))
            # Never mask real scan endpoints, even when a held object blocks the laser.
            hits.append(point)
        return hits

    def publish_working_map(self):
        msg = deepcopy(self.static_map)
        current = self.get_clock().now().nanoseconds
        if current <= 0:
            return False
        if self.grid_generation is not None and current <= self.grid_generation:
            return False
        # First inspect the static-map route, then compare detour and removal.
        # Overlaying the target before this decision would silently force detours.
        use_dynamic = self.nav_purpose != 'MISSION'
        obstacles = list(self.known.values()) if use_dynamic else []
        excluded = (self.backend.held_id,) if self.backend and self.backend.held_id else ()
        hits = self.scan_hits() if self.scan_view and use_dynamic else []
        msg.data = self.grid_model.overlay(obstacles, hits, exclude_ids=excluded)
        if self.backend and self.backend.held_id and self.chosen and self.chosen.extra_radius > 0:
            # planning_grid parameters are read-only. Expand source obstacles by
            # the additional carried radius, then retain normal base inflation.
            # Double raster compensation is conservative, never a smaller envelope.
            import _planning_eval
            extra_config = {**self.grid_model.config,
                            'robot_radius': self.chosen.extra_radius, 'safety_margin': 0.0}
            expanded = _planning_eval.Grid(self.grid_model.geometry, list(msg.data),
                                            extra_config).inflated
            msg.data = [100 if v == 100 else raw for raw, v in zip(msg.data, expanded)]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.grid_generation = stamp_ns(msg.header.stamp)
        self.grid = None
        self.working_snapshot = msg
        self.working_model = GridModel.from_message(msg, self.config.get('planning', {}))
        self.working_pub.publish(msg)
        return True

    def prepare_plan(self):
        self.refresh_observation()
        if stamp_ns(self.scan.header.stamp) <= self.minimum_scan_stamp:
            if self.nav_state != 'WAIT_OPERATION_SCAN':
                self.transition('WAIT_OPERATION_SCAN')
            return
        self.candidate_path = None
        self.planner_success = False
        self.path_cleared = False
        if self.publish_working_map():
            self.transition('WAIT_GRID')
        else:
            self.transition('PREPARE_MAP')

    def send_planning_goal(self):
        stamp = self.get_clock().now().to_msg()
        value = stamp_ns(stamp)
        if value <= self.last_request_stamp:
            return
        self.last_request_stamp = value
        self.goal_stamp = value
        self.goal_pub.publish(pose_message(self.nav_target, self.frame, stamp))
        self.transition('WAIT_PLAN')

    def on_diagnostics(self, source, msg):
        for status in msg.status:
            values = {item.key: item.value for item in status.values}
            self.remember(source + '_diagnostic', dict(state=status.message, **values))
            if self.mode != 'ACTIVE':
                continue
            if source == 'planner' and self.goal_stamp is not None:
                if values.get('goal_stamp_ns') != str(self.goal_stamp):
                    continue
                if status.message == 'SUCCESS':
                    self.planner_success = values.get('path_stamp_ns') == str(self.goal_stamp)
                else:
                    self.fail(status.message, values.get('detail', 'Planner rejected goal'))
            elif source == 'lqr' and self.execution_stamp is not None:
                if values.get('path_stamp_ns') != str(self.execution_stamp):
                    continue
                if status.message in ('TRACKING_ERROR', 'PATH_INVALID', 'PATH_UNAVAILABLE',
                                      'TF_UNAVAILABLE', 'ODOM_UNAVAILABLE', 'CLOCK_UNAVAILABLE'):
                    self.fail(status.message, values.get('detail', 'LQR failure'))

    def on_path(self, msg):
        if self.mode != 'ACTIVE' or self.nav_state != 'WAIT_PLAN' or self.goal_stamp is None:
            return
        if not msg.poses:
            if stamp_ns(msg.header.stamp) >= self.goal_stamp:
                self.path_cleared = True
            return
        if stamp_ns(msg.header.stamp) != self.goal_stamp:
            return
        try:
            if msg.header.frame_id != self.frame:
                raise ValueError('Candidate frame mismatch')
            expected = Waypoint('navigation_subgoal', self.nav_target.x,
                                self.nav_target.y, self.nav_target.yaw)
            x, y = self.grid_endpoint(expected)
            end = msg.poses[-1].pose
            if (math.hypot(end.position.x-x, end.position.y-y) > 1e-6 or
                    abs(angle_error(yaw_of(end.orientation), self.nav_target.yaw)) > 1e-6):
                raise ValueError('Candidate endpoint does not match navigation subgoal')
            points = []
            for item in msg.poses:
                if item.header.frame_id != self.frame:
                    raise ValueError('Candidate pose frame mismatch')
                pose = item.pose
                p = Pose3(pose.position.x, pose.position.y, pose.position.z,
                          yaw_of(pose.orientation))
                if not all(math.isfinite(v) for v in asdict(p).values()):
                    raise ValueError('Nonfinite candidate pose')
                points.append(p)
            self.candidate_path = msg
            self.segments = split_path(points)
            if any(not self.working_model.line_clear(segment[0], segment[-1])
                   for segment in self.segments):
                raise ValueError('Candidate intersects current working map snapshot')
            if not self.segments:
                self.segments = [[points[0], points[0]]]
            self.segment_index = 0
            self.route_cursor = self.segments[0][0]
        except (ValueError, RuntimeError) as error:
            self.fail('PATH_MISMATCH', str(error))

    def radius(self):
        p = self.config.get('planning', {})
        value = p.get('robot_radius', 0.355) + p.get('safety_margin', 0.10)
        if self.backend.held_id:
            held = self.known.get(self.backend.held_id)
            if held:
                offset = math.hypot(self.backend_config.get('carry_x', 0.0),
                                    self.backend_config.get('carry_y', 0.0))
                value = max(value, offset + math.hypot(*held.size[:2])/2 +
                            p.get('safety_margin', 0.10))
        return value

    def inspect(self):
        self.refresh_observation()
        start = self.base_pose()
        end = self.segments[self.segment_index][-1]
        result = self.scan_view.inspect_segment(
            start, end, self.radius(), spacing=self.n['corridor_spacing'],
            stop_margin=self.n['stop_margin'],
            self_clear_radius=self.config.get('planning', {}).get('robot_radius', 0.355))
        if result.state == 'blocked':
            self.stop_then(lambda: self.handle_blockage(result, start, end))
            return
        distance = pose_distance(start, end)
        extent = min(distance, self.n['observation_chunk_length'])
        if result.state == 'unknown':
            extent = min(extent, result.clear_distance)
        if extent < self.n['min_advance'] and distance > self.p['position_tolerance']:
            if time.monotonic() - self.state_started > self.n['inspection_timeout_sec']:
                self.fail('VISIBILITY_UNKNOWN', 'Cannot admit a stopping-distance observation horizon')
            return
        # Endpoints remain on the accepted straight segment, including quantized starts.
        segment_start = self.route_cursor
        remaining = pose_distance(segment_start, end)
        if remaining > extent + self.p['position_tolerance'] and extent >= self.n['min_advance']:
            ratio = extent / remaining
            heading = math.atan2(end.y-segment_start.y, end.x-segment_start.x)
            target = Pose3(segment_start.x + ratio*(end.x-segment_start.x),
                           segment_start.y + ratio*(end.y-segment_start.y), end.z, heading)
            self.chunk_is_segment_end = False
        else:
            target = end
            self.chunk_is_segment_end = True
        self.pending_chunk = [segment_start, target]
        self.nav_future = self.inhibit.call_async(SetBool.Request(data=False))
        self.nav_ack_time = None
        self.transition('SEGMENT_RELEASING')

    def publish_chunk(self):
        now = self.get_clock().now()
        if now.nanoseconds <= max(self.release_ros, self.last_request_stamp):
            return
        # Reinspect immediately after release; releasing the controller alone cannot move it.
        self.refresh_observation()
        check = self.scan_view.inspect_segment(
            self.base_pose(), self.pending_chunk[-1], self.radius(),
            spacing=self.n['corridor_spacing'], stop_margin=self.n['stop_margin'],
            self_clear_radius=self.config.get('planning', {}).get('robot_radius', 0.355))
        if check.state != 'clear':
            self.stop_then(lambda: self.transition('INSPECT_SEGMENT'))
            return
        path = Path()
        path.header.frame_id, path.header.stamp = self.frame, now.to_msg()
        path.poses = [pose_message(p, self.frame, path.header.stamp) for p in self.pending_chunk]
        self.execution_stamp = now.nanoseconds
        self.last_request_stamp = now.nanoseconds
        self.active_path = path
        self.execution_pub.publish(path)
        self.motion_inhibited = False
        self.stop_confirmed = False
        self.transition('DRIVE_SEGMENT')
        self.event('SEGMENT_ADMITTED', segment_index=self.segment_index,
                   execution_stamp_ns=self.execution_stamp,
                   target=asdict(self.pending_chunk[-1]))

    def on_reached(self, msg):
        if (self.mode != 'ACTIVE' or self.nav_state != 'DRIVE_SEGMENT' or
                self.execution_stamp is None or not self.active_path or
                stamp_ns(msg.header.stamp) != self.execution_stamp or
                msg.header.frame_id != self.frame or
                msg.pose != self.active_path.poses[-1].pose):
            return
        try:
            if not self.at_endpoint():
                return
            self.execution_stamp = None
            self.route_cursor = self.pending_chunk[-1]
            self.stop_then(self.chunk_completed)
        except (ValueError, TransformException) as error:
            self.fail('ARRIVAL_INPUT_INVALID', str(error))

    def chunk_completed(self):
        self.event('SEGMENT_STOPPED', segment_index=self.segment_index)
        if not self.chunk_is_segment_end:
            self.transition('INSPECT_SEGMENT')
            return
        self.segment_index += 1
        if self.segment_index < len(self.segments):
            self.route_cursor = self.segments[self.segment_index][0]
            self.transition('INSPECT_SEGMENT')
            return
        if self.nav_purpose in ('MISSION', 'DETOUR'):
            self.chosen = None
            self.effects(self.engine.reached(self.get_clock().now().nanoseconds / 1e9))
        elif self.nav_purpose == 'ACTION':
            self.begin_grasp()
        elif self.nav_purpose == 'RESTORE_BASE':
            if self.chosen.kind == 'temporary':
                self.begin_grasp(regrasp=True)
            else:
                self.begin_place(self.chosen.original_pose, restore=True)
        elif self.nav_purpose == 'ROLLBACK':
            self.begin_place(self.chosen.original_pose, restore=True)
        elif self.nav_purpose == 'ROLLBACK_REGRASP':
            self.begin_grasp(regrasp=True)

    def handle_blockage(self, result, start, end):
        self.refresh_observation()
        if self.pending_failure:
            self.hard_fail('ROLLBACK_BLOCKED', 'A new obstacle blocks the restoration rollback')
            return
        if self.backend.held_id or (self.chosen and self.ledger.items):
            self.fail('SECOND_OBSTACLE', 'New blockage while an object is displaced')
            return
        self.recovery_cycles += 1
        if self.recovery_cycles > self.n['max_recovery_cycles']:
            self.fail('RECOVERY_LIMIT', 'Bounded recovery cycle budget exhausted')
            return
        length = max(pose_distance(start, end), 1e-9)
        dx, dy = (end.x-start.x)/length, (end.y-start.y)/length
        obstacles = []
        for obstacle in self.known.values():
            along = (obstacle.pose.x-start.x)*dx + (obstacle.pose.y-start.y)*dy
            lateral = abs((obstacle.pose.x-start.x)*dy - (obstacle.pose.y-start.y)*dx)
            half = math.hypot(*obstacle.size[:2])/2
            if (-half <= along <= length + self.n['stop_margin'] + half and
                    lateral <= self.radius() + half):
                obstacles.append((along, obstacle))
        if obstacles:
            obstacle = min(obstacles, key=lambda entry: entry[0])[1]
        else:
            points = result.blocking_points
            if not points:
                self.fail('UNCLASSIFIED_BLOCKAGE', 'No usable obstruction geometry')
                return
            point = points[0]
            x, y = (point.x, point.y) if hasattr(point, 'x') else point[:2]
            size = self.n['anonymous_obstacle_size']
            obstacle = Obstacle(f'unknown_{self.recovery_cycles}', 'unknown',
                                Pose3(x, y, size/2), (size, size, size), 0.0,
                                stamp_ns(self.scan.header.stamp), self.frame)
            self.known[obstacle.object_id] = obstacle
        mission = self.engine.target
        goal = Pose3(mission.x, mission.y, 0.0, mission.yaw)
        # Include unclassified scan hits in the static evaluation snapshot as obstacles.
        # Remove target hit cells only in hypotheses where its measured body is removed.
        self.evaluation_context = (obstacle.object_id, goal)
        base = self.base_pose()
        known, hits = list(self.known.values()), self.scan_hits()
        excluded, now_ns = set(self.excluded), self.get_clock().now().nanoseconds
        grid, config = self.grid_model, self.config

        def evaluate_snapshot():
            evaluation_grid = grid.with_unclassified_hits(hits, known)
            return RecoveryPlanner(config, evaluation_grid).choose(
                base, goal, obstacle, known, excluded, now_ns)

        self.evaluation_future = self.evaluator.submit(
            evaluate_snapshot)
        self.transition('EVALUATE_SCHEMES')

    def accept_evaluation(self):
        chosen, evaluations = self.evaluation_future.result()
        self.evaluation_future = None
        object_id, goal = self.evaluation_context
        if not self.event('SCHEMES_EVALUATED', object_id=object_id,
                          evaluations=evaluations):
            return
        if chosen is None:
            self.fail('NO_SCHEME', 'No feasible detour or restorable manipulation scheme')
            return
        self.chosen = chosen
        self.grasp_attempts = 0
        if not self.event('SCHEME_SELECTED', candidate=asdict(chosen)):
            return
        self.publish_markers()
        if chosen.kind == 'detour':
            self.plan_to(goal, 'DETOUR')
        else:
            self.plan_to(chosen.action_pose, 'ACTION')

    def publish_markers(self):
        markers = []
        for index, key in enumerate(('action_pose', 'pass_pose', 'restore_pose',
                                     'temporary_pose', 'original_pose')):
            pose = getattr(self.chosen, key, None)
            if pose is None:
                continue
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = key, index, Marker.ARROW
            marker.action = Marker.ADD
            marker.pose = pose_message(pose, self.frame, marker.header.stamp).pose
            marker.scale.x, marker.scale.y, marker.scale.z = 0.25, 0.05, 0.05
            marker.color.a, marker.color.g, marker.color.b = 1.0, 0.8, 0.5
            markers.append(marker)
        self.marker_pub.publish(MarkerArray(markers=markers))

    def begin_grasp(self, regrasp=False):
        observation = self.backend.get_observation(self.chosen.object_id)
        if observation is None:
            self.fail('OBJECT_LOST', 'Manipulation target has no fresh authorized observation')
            return
        expected = self.chosen.temporary_pose if regrasp else self.chosen.original_pose
        position_error = math.sqrt((observation.pose.x-expected.x)**2 +
                                   (observation.pose.y-expected.y)**2 +
                                   (observation.pose.z-expected.z)**2)
        if (position_error > self.backend_config.get('restore_position_tolerance', 0.04) or
                abs(angle_error(observation.pose.yaw, expected.yaw)) >
                self.backend_config.get('restore_yaw_tolerance', 0.08)):
            self.fail('OBJECT_MOVED', 'Target moved after scheme evaluation; refusing stale grasp')
            return
        evaluator = RecoveryPlanner(self.config, self.grid_model)
        if not evaluator._reachable(self.base_pose(), observation.pose):
            self.fail('ACTION_UNREACHABLE', 'Actual quantized base pose cannot reach target')
            return
        self.ledger.add(observation, self.task_id)
        self.grasp_attempts += 1
        self.regrasp = regrasp
        if not self.backend.begin_grasp(observation) and self.backend.result is None:
            self.hard_fail('MANIPULATION_BUSY', 'Backend rejected grasp without a result')
            return
        self.operation_pose = self.base_pose()
        self.transition('REGRASP' if regrasp else 'GRASP')
        self.event('GRASP_STARTED', object_id=observation.object_id, attempt=self.grasp_attempts)

    def begin_place(self, target, restore=False):
        observation = self.backend.get_observation(self.chosen.object_id)
        if observation is None:
            self.fail('RESTORE_FAILED', 'No fresh held-object pose before placement')
            return
        known = list(self.known.values())
        evaluation_grid = self.grid_model.with_unclassified_hits(self.scan_hits(), known)
        evaluator = RecoveryPlanner(self.config, evaluation_grid)
        if not evaluator._reachable(self.base_pose(), target):
            self.fail('RESTORE_FAILED', 'Actual base pose cannot reach placement target')
            return
        if not evaluator._sweep_clear(observation.pose, target, observation, known):
            self.fail('RESTORE_FAILED', 'Fresh obstacles block the placement sweep')
            return
        if not self.backend.begin_place(target, restore=restore) and self.backend.result is None:
            self.hard_fail('MANIPULATION_BUSY', 'Backend rejected placement without a result')
            return
        self.operation_pose = self.base_pose()
        self.transition('RESTORE' if restore else 'TEMPORARY_PLACE')
        self.event('RESTORE_STARTED' if restore else 'TEMPORARY_PLACE_STARTED',
                   object_id=self.chosen.object_id, pose=asdict(target))

    def operation_tick(self):
        current = self.base_pose()
        if (not self.stopped() or pose_distance(current, self.operation_pose) >
                self.p['position_tolerance'] or
                abs(angle_error(current.yaw, self.operation_pose.yaw)) >
                self.p['heading_tolerance']):
            self.hard_fail('ACTION_POSE_LOST', 'Base moved during manipulation')
            return
        result = self.backend.result
        if result is None:
            if time.monotonic() - self.state_started > self.n['operation_timeout_sec']:
                self.hard_fail('OPERATION_TIMEOUT', 'Manipulation feedback deadline expired')
            return
        self.backend.result = None
        if not result.get('success'):
            self.event('OPERATION_FAILED', result=result, attempt=self.grasp_attempts)
            if self.nav_state in ('GRASP', 'REGRASP') and not self.backend.held_id:
                if self.grasp_attempts < self.n['max_grasp_attempts']:
                    self.begin_grasp(self.regrasp)
                    return
                self.excluded.add((self.chosen.object_id, self.chosen.kind))
                self.transition('VERIFY_UNMOVED')
            else:
                self.fail('RESTORE_FAILED', result.get('detail', 'Manipulation failed'))
            return
        self.event('OPERATION_VERIFIED', result=result)
        self.minimum_scan_stamp = self.get_clock().now().nanoseconds
        observation = self.backend.get_observation(self.chosen.object_id)
        if observation:
            self.known[observation.object_id] = observation
        if self.nav_state == 'GRASP':
            if self.chosen.kind == 'temporary':
                self.transition('WAIT_MANIPULATION_SCAN')
            else:
                self.plan_to(self.chosen.restore_pose, 'RESTORE_BASE')
        elif self.nav_state == 'TEMPORARY_PLACE':
            self.grasp_attempts = 0
            self.plan_to(self.chosen.restore_pose, 'RESTORE_BASE')
        elif self.nav_state == 'REGRASP':
            self.begin_place(self.chosen.original_pose, restore=True)
        else:
            self.ledger.remove(self.chosen.object_id)
            self.event('OBJECT_RESTORED', object_id=self.chosen.object_id,
                       original_pose=asdict(self.chosen.original_pose))
            if self.pending_failure:
                code, detail = self.pending_failure
                self.pending_failure = None
                self.hard_fail(code, detail + '; displaced object restored')
            else:
                self.chosen = None
                target = self.engine.target
                self.plan_to(Pose3(target.x, target.y, 0.0, target.yaw), 'MISSION')

    def hard_fail(self, code, detail):
        self.execution_stamp = None
        if self.backend:
            self.backend.cancel()
        super().fail(code, detail)

    def fail(self, code, detail):
        if self.mode in ('STOPPING', 'FAILED'):
            return
        displaced = (self.backend and (self.backend.held_id or
                     (self.chosen and self.chosen.kind == 'temporary' and self.ledger.items)))
        can_rollback = (self.mode == 'ACTIVE' and self.chosen and displaced and
                        not self.pending_failure and
                        code in ('SECOND_OBSTACLE', 'VISIBILITY_UNKNOWN', 'NO_SCHEME',
                                 'RESTORE_FAILED', 'NO_PATH'))
        if can_rollback:
            self.pending_failure = (code, detail)
            self.backend.cancel()
            self.backend.result = None
            self.event('ROLLBACK_REQUESTED', reason=dict(code=code, detail=detail))
            self.plan_to(self.chosen.action_pose,
                         'ROLLBACK' if self.backend.held_id else 'ROLLBACK_REGRASP')
        else:
            self.hard_fail(code, detail)

    def navigation_tick(self):
        now = time.monotonic()
        elapsed = now - self.state_started
        if self.nav_state == 'NAV_STOPPING':
            if self.nav_future.done():
                result = self.nav_future.result()
                if result is None or not result.success:
                    self.hard_fail('INHIBIT_SERVICE_FAILED', 'Navigation stop was rejected')
                    return
                if self.nav_ack_time is None:
                    self.nav_ack_time = now
                if self.odom_received >= self.nav_ack_time and self.stopped():
                    self.motion_inhibited, self.stop_confirmed = True, True
                    callback, self.stop_continuation = self.stop_continuation, None
                    callback()
                    return
            if elapsed > self.p['stop_timeout_sec']:
                self.hard_fail('STOP_UNCONFIRMED', 'Navigation pause could not confirm stopped base')
        elif self.nav_state == 'PREPARE_MAP':
            self.prepare_plan()
        elif self.nav_state == 'EVALUATE_SCHEMES':
            if self.evaluation_future.done():
                self.accept_evaluation()
            elif elapsed > self.n['operation_timeout_sec']:
                self.hard_fail('EVALUATION_TIMEOUT', 'Candidate evaluation exceeded its deadline')
        elif self.nav_state == 'WAIT_OPERATION_SCAN':
            if stamp_ns(self.scan.header.stamp) > self.minimum_scan_stamp:
                self.prepare_plan()
            elif elapsed > self.n['inspection_timeout_sec']:
                self.hard_fail('SCAN_UNAVAILABLE', 'No post-manipulation scan')
        elif self.nav_state == 'WAIT_MANIPULATION_SCAN':
            if stamp_ns(self.scan.header.stamp) > self.minimum_scan_stamp:
                self.begin_place(self.chosen.temporary_pose)
            elif elapsed > self.n['inspection_timeout_sec']:
                self.hard_fail('SCAN_UNAVAILABLE', 'No post-lift scan before temporary placement')
        elif self.nav_state == 'VERIFY_UNMOVED':
            self.verify_ledger()
            if not self.ledger.items:
                self.chosen = None
                target = self.engine.target
                self.plan_to(Pose3(target.x, target.y, 0.0, target.yaw), 'MISSION')
            elif elapsed > self.n['operation_timeout_sec']:
                self.hard_fail('GRASP_EXHAUSTED', 'Target restoration is not confirmed')
        elif self.nav_state == 'WAIT_GRID':
            if self.grid and now - self.grid_ack_time >= self.n['map_settle_sec']:
                self.send_planning_goal()
            elif elapsed > self.p['planning_timeout_sec']:
                self.fail('MAP_UPDATE_TIMEOUT', 'No matching inflated snapshot')
        elif self.nav_state == 'WAIT_PLAN':
            if self.candidate_path and self.planner_success:
                self.transition('INSPECT_SEGMENT')
            elif self.path_cleared:
                self.fail('PATH_CLEARED', 'Planner cleared current candidate; inspect diagnostics')
            elif elapsed > self.p['planning_timeout_sec']:
                self.fail('PLANNING_TIMEOUT', 'No correlated candidate and planning success')
        elif self.nav_state == 'INSPECT_SEGMENT':
            self.inspect()
        elif self.nav_state == 'SEGMENT_RELEASING':
            if self.nav_future.done():
                result = self.nav_future.result()
                if result is None or not result.success:
                    self.fail('INHIBIT_SERVICE_FAILED', 'Segment release rejected')
                    return
                if self.nav_ack_time is None:
                    self.nav_ack_time = now
                    self.release_ros = self.get_clock().now().nanoseconds
                self.publish_chunk()
            elif elapsed > self.p['preparation_timeout_sec']:
                self.fail('RELEASE_TIMEOUT', 'Segment release not acknowledged')
        elif self.nav_state == 'DRIVE_SEGMENT':
            check = self.scan_view.inspect_segment(
                self.base_pose(), self.pending_chunk[-1], self.radius(),
                spacing=self.n['corridor_spacing'], stop_margin=self.n['stop_margin'],
                self_clear_radius=self.config.get('planning', {}).get('robot_radius', 0.355))
            if check.state != 'clear':
                if check.state == 'blocked':
                    start, end = self.base_pose(), self.pending_chunk[-1]
                    self.stop_then(lambda: self.handle_blockage(check, start, end))
                else:
                    self.stop_then(lambda: self.transition('INSPECT_SEGMENT'))
            elif elapsed > self.p['navigation_timeout_sec']:
                self.fail('NAVIGATION_TIMEOUT', 'Segment did not complete before deadline')
        elif self.nav_state in ('GRASP', 'REGRASP', 'TEMPORARY_PLACE', 'RESTORE'):
            self.operation_tick()

    def tick_checked(self):
        if self.backend:
            self.backend.tick()
        if self.mode in ('IDLE', 'FAILED'):
            try:
                self.refresh_observation()
                self.verify_ledger()
            except (ValueError, TransformException):
                pass
            if self.static_map is not None and self.grid_generation is None:
                self.publish_working_map()
        if self.mode != 'ACTIVE':
            super().tick_checked()
            return
        now = self.get_clock().now().nanoseconds
        wall = time.monotonic()
        if self.log_broken:
            self.hard_fail('LOG_WRITE_FAILED', 'Task journal unavailable')
            return
        if self.last_ros is not None and now < self.last_ros:
            self.hard_fail('CLOCK_RESET', 'ROS clock moved backwards')
            return
        if now != self.last_ros:
            self.last_clock_progress = wall
        elif wall - self.last_clock_progress > self.p['input_timeout_sec']:
            self.hard_fail('CLOCK_UNAVAILABLE', 'ROS clock stopped')
            return
        self.last_ros = now
        self.ready()
        if self.engine.state.startswith('NAV_'):
            self.navigation_tick()
        elif self.engine.state in ('PICKING', 'DROPPING'):
            if not self.at_endpoint():
                self.hard_fail('ACTION_POSE_LOST', 'Mission action pose or stop confirmation lost')
                return
            self.effects(self.engine.tick(now / 1e9))

    def tick(self):
        try:
            self.tick_checked()
        except (ValueError, OSError, RuntimeError, TransformException, OverflowError) as error:
            if self.mode in ('ACTIVE', 'PREPARING', 'RELEASING'):
                prefix = str(error).split(':', 1)[0]
                code = prefix if prefix.endswith('_UNAVAILABLE') else 'RECOVERY_INPUT_INVALID'
                self.hard_fail(code, str(error))
            else:
                self.get_logger().error(f'Recovery input error: {error}')
        self.publish_status()

    def destroy_node(self):
        self.evaluator.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = RecoveryTaskManager()
    shutting_down = False

    def shutdown_request(signum, frame):
        nonlocal shutting_down
        shutting_down = True

    signal.signal(signal.SIGINT, shutdown_request)
    signal.signal(signal.SIGTERM, shutdown_request)
    try:
        while rclpy.ok() and not shutting_down:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if rclpy.ok():
            if node.mode in ('ACTIVE', 'PREPARING', 'RELEASING', 'STOPPING'):
                node.hard_fail('SHUTDOWN', 'Shutdown requested; preserve restoration ledger')
                deadline = time.monotonic() + node.p['stop_timeout_sec']
                while time.monotonic() < deadline and not node.stop_confirmed:
                    rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
            rclpy.shutdown()

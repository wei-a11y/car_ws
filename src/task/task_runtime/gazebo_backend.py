# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Asynchronous Gazebo-only perception and virtual gripper backend.

Entity poses are measured relative to a real Gazebo reference link, then
transformed using TF at the entity response timestamp.  No map/world identity
is assumed.  Commands move registered test models, never real arm joints.
"""
from dataclasses import dataclass
import math
import time

from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Pose
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from task_runtime.recovery_core import Obstacle, Pose3


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def quaternion_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(x) for x in values) or abs(math.hypot(*values) - 1) > 1e-4:
        raise ValueError('Invalid entity quaternion')
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def rotate(q, v):
    """Rotate a 3-vector by a normalized quaternion, without a numpy dependency."""
    quaternion_yaw(q)  # Also validate all quaternion components.
    x, y, z = v
    tx, ty, tz = 2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x)
    return (x + q.w*tx + q.y*tz - q.z*ty,
            y + q.w*ty + q.z*tx - q.x*tz,
            z + q.w*tz + q.x*ty - q.y*tx)


def transform_pose(pose, transform, inverse=False):
    """Transform a planar-yaw object pose, preserving the full 3D translation."""
    t, q = transform.translation, transform.rotation
    if inverse:
        from geometry_msgs.msg import Quaternion
        conjugate = Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)
        x, y, z = rotate(conjugate, (pose.x-t.x, pose.y-t.y, pose.z-t.z))
        yaw = pose.yaw - quaternion_yaw(q)
    else:
        x, y, z = rotate(q, (pose.x, pose.y, pose.z))
        x, y, z = x+t.x, y+t.y, z+t.z
        yaw = pose.yaw + quaternion_yaw(q)
    return Pose3(x, y, z, math.remainder(yaw, 2*math.pi))


def ros_pose(pose):
    result = Pose()
    result.position.x, result.position.y, result.position.z = pose.x, pose.y, pose.z
    result.orientation.z, result.orientation.w = math.sin(pose.yaw/2), math.cos(pose.yaw/2)
    return result


def distance(a, b):
    return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)


def interpolate(a, b, step):
    d = distance(a, b)
    fraction = 1.0 if d <= step else step/d
    return Pose3(a.x + (b.x-a.x)*fraction, a.y + (b.y-a.y)*fraction,
                 a.z + (b.z-a.z)*fraction,
                 a.yaw + math.remainder(b.yaw-a.yaw, 2*math.pi)*fraction)


def load_registry(path):
    with open(path, encoding='utf-8') as stream:
        source = yaml.safe_load(stream)
    if not isinstance(source, dict) or not isinstance(source.get('objects'), list):
        raise ValueError('Scenario registry requires an objects list')
    registry, names = {}, set()
    for entry in source['objects']:
        item = dict(entry)
        identity, name = item.get('object_id'), item.get('model_name')
        if not isinstance(identity, str) or not identity or identity in registry:
            raise ValueError('Scenario object IDs must be unique, nonempty strings')
        if not isinstance(name, str) or not name.startswith('task_recovery_') or name in names:
            raise ValueError('Models must have unique task_recovery_ prefixed names')
        if not isinstance(item.get('label'), str) or not item['label']:
            raise ValueError('Each registry object requires a class label')
        size = item.get('size')
        if not isinstance(size, list) or len(size) != 3 or any(
                not isinstance(x, (int, float)) or not math.isfinite(x) or x <= 0 for x in size):
            raise ValueError('Object size requires three positive dimensions')
        confidence = float(item.get('confidence', 1.0))
        if not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('Object confidence must lie in [0, 1]')
        item['confidence'] = confidence
        item['grasp_failures'] = int(item.get('grasp_failures', 0))
        if item['grasp_failures'] < 0:
            raise ValueError('grasp_failures must be nonnegative')
        registry[identity], names = item, names | {name}
    return registry


@dataclass
class Pending:
    future: object
    started: float
    response: object = None


class GazeboBackend:
    """Nonblocking simulated manipulation with observation-based verification.

    ``observations`` never exposes hidden registry poses: its caller supplies a
    scan-visibility predicate. ``get_observation`` is reserved for the target
    currently manipulated or held. Call ``tick`` from the node's steady timer.
    """

    DEFAULTS = dict(
        planning_frame='map', reference_frame='base_link', reference_entity='car::base_link',
        get_state_service='/task/gazebo/get_entity_state',
        set_state_service='/task/gazebo/set_entity_state',
        backend_poll_sec=0.1, backend_timeout_sec=2.0, observation_timeout_sec=0.6,
        action_step_sec=0.2, lift_height=0.25, lift_speed=0.15,
        lift_min_displacement=0.15, follow_tolerance=0.06, verification_samples=3,
        restore_position_tolerance=0.04, restore_yaw_tolerance=0.08,
        carry_x=0.65, carry_y=0.0, carry_z=0.8, arm_reach=1.25,
        pregrasp_offset=0.12, marker_topic='/task/recovery/markers')

    def __init__(self, node, config, buffer):
        self.node, self.buffer = node, buffer
        self.config = {**self.DEFAULTS, **config}
        self.registry = load_registry(self.config['backend_registry_file'])
        for key in ('backend_poll_sec', 'backend_timeout_sec', 'observation_timeout_sec',
                    'action_step_sec', 'lift_height', 'lift_speed', 'lift_min_displacement',
                    'follow_tolerance', 'verification_samples', 'restore_position_tolerance',
                    'restore_yaw_tolerance', 'arm_reach', 'pregrasp_offset'):
            if not math.isfinite(self.config[key]) or self.config[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.config['lift_min_displacement'] > self.config['lift_height']:
            raise ValueError('lift_min_displacement must not exceed lift_height')
        self.frame = self.config['planning_frame']
        self.reference = self.config['reference_frame']
        self.reference_entity = self.config['reference_entity']
        self.get_client = node.create_client(GetEntityState, self.config['get_state_service'])
        self.set_client = node.create_client(SetEntityState, self.config['set_state_service'])
        self.marker_pub = node.create_publisher(MarkerArray, self.config['marker_topic'], 10)
        self.cache, self.pending, self.authorized = {}, {}, {}
        self.last_poll = self.last_marker = -math.inf
        self.reference_stamp = 0
        self.reference_wall = -math.inf
        self.last_error = 'Awaiting reference entity and timestamped TF'
        self.set_pending = None
        self.operation, self.result = 'IDLE', None
        self.held_id = self.target_id = None
        self.tcp = self.held_relative = None
        self.original = self.goal = None
        self.gripper_open = True
        self.step_at = self.operation_at = 0.0
        self.last_tick = time.monotonic()
        self.last_ros = None
        self.clock_progress_at = self.last_tick
        self.last_verified_stamp = self.verify_after = 0
        self.verify_count = 0
        self.injected = False
        self.attempts = {}
        self.restoring = False

    def _now(self):
        return self.node.get_clock().now().nanoseconds

    def _fresh(self, stamp, wall=None):
        age = (self._now() - stamp) / 1e9
        return 0 <= age <= self.config['observation_timeout_sec'] and (
            wall is None or time.monotonic()-wall <= self.config['observation_timeout_sec'])

    def _tf(self, stamp=None):
        value = self.buffer.lookup_transform(
            self.frame, self.reference,
            Time(nanoseconds=stamp) if stamp is not None else Time())
        tf_stamp = stamp_ns(value.header.stamp)
        if tf_stamp and not self._fresh(tf_stamp):
            raise ValueError('Stale reference TF')
        return value.transform

    def ready(self):
        if not self.get_client.service_is_ready() or not self.set_client.service_is_ready():
            return False, 'Gazebo state services unavailable'
        if not self._fresh(self.reference_stamp, self.reference_wall):
            return False, self.last_error or 'Stale reference entity / TF'
        if time.monotonic()-self.clock_progress_at > self.config['backend_timeout_sec']:
            return False, 'Simulation clock is not advancing'
        return True, ''

    def observations(self, visibility_callback):
        visible = []
        for identity, (obstacle, received) in self.cache.items():
            if identity == self.held_id or not self._fresh(obstacle.stamp_ns, received):
                continue
            if visibility_callback(obstacle):
                visible.append(obstacle)
                self.authorized[identity] = time.monotonic()
        return visible

    def get_observation(self, identity):
        authorized = identity in self.authorized and (
            time.monotonic() - self.authorized[identity] <= self.config['observation_timeout_sec'])
        if identity not in (self.target_id, self.held_id) and not authorized:
            return None
        entry = self.cache.get(identity)
        return entry[0] if entry and self._fresh(entry[0].stamp_ns, entry[1]) else None

    def _change(self, operation):
        self.operation, self.step_at = operation, time.monotonic()

    def _finish(self, success, code, detail=''):
        self.result = dict(success=bool(success), code=code, detail=detail)
        self.operation = 'IDLE'
        if not self.held_id:
            self.target_id = None

    def begin_grasp(self, obstacle, now=None):
        if self.operation != 'IDLE' or self.held_id:
            return False
        if (obstacle.object_id not in self.authorized or obstacle.object_id not in self.registry or
                time.monotonic()-self.authorized[obstacle.object_id] > self.config['observation_timeout_sec']):
            self._finish(False, 'UNAUTHORIZED_OBJECT', 'Target must be freshly visible and registered')
            return False
        cached = self.cache.get(obstacle.object_id)
        if not cached or not self._fresh(cached[0].stamp_ns, cached[1]):
            self._finish(False, 'STALE_OBJECT')
            return False
        if not self.ready()[0]:
            self._finish(False, 'BACKEND_NOT_READY', self.ready()[1])
            return False
        try:
            relative = transform_pose(cached[0].pose, self._tf(cached[0].stamp_ns), True)
            if math.hypot(relative.x, relative.y) > self.config['arm_reach']:
                self._finish(False, 'UNREACHABLE_GRASP')
                return False
        except Exception as error:
            self._finish(False, 'TF_UNAVAILABLE', str(error))
            return False
        self.target_id = obstacle.object_id
        self.original = cached[0].pose
        self.goal = Pose3(self.original.x, self.original.y,
                          self.original.z+self.config['lift_height'], self.original.yaw)
        self.tcp = Pose3(self.original.x, self.original.y,
                         self.original.z+self.config['pregrasp_offset'], self.original.yaw)
        self.result = None
        self.operation_at = time.monotonic()
        self.attempts[self.target_id] = self.attempts.get(self.target_id, 0)+1
        self.injected = self.attempts[self.target_id] <= self.registry[self.target_id]['grasp_failures']
        self.verify_count = self.last_verified_stamp = 0
        self._change('PREGRASP')
        return True

    def begin_place(self, target, restore=False):
        if self.operation != 'IDLE' or not self.held_id:
            return False
        if not all(math.isfinite(x) for x in (target.x, target.y, target.z, target.yaw)):
            self._finish(False, 'INVALID_PLACE_POSE')
            return False
        try:
            relative = transform_pose(target, self._tf(), True)
            if math.hypot(relative.x, relative.y) > self.config['arm_reach']:
                self._finish(False, 'UNREACHABLE_PLACE')
                return False
        except Exception as error:
            self._finish(False, 'TF_UNAVAILABLE', str(error))
            return False
        observation = self.get_observation(self.held_id)
        if observation is None:
            self._finish(False, 'STALE_HELD_OBJECT')
            return False
        self.target_id, self.goal, self.restoring = self.held_id, target, restore
        self.tcp = observation.pose
        self.result = None
        self.operation_at = time.monotonic()
        self.verify_count = self.last_verified_stamp = 0
        self._change('PLACE_APPROACH')
        return True

    def cancel(self):
        # Gazebo services cannot be un-sent. A pending command is at most one
        # velocity-limited step; no further trajectory step is submitted.
        if self.held_id and self.tcp is not None:
            try:
                self.held_relative = transform_pose(self.tcp, self._tf(), True)
            except Exception:
                self.held_relative = None
        self._finish(False, 'CANCELLED', 'Trajectory stopped; held object remains attached')

    def _request_states(self):
        for identity in ['__reference__'] + list(self.registry):
            if identity in self.pending:
                continue
            request = GetEntityState.Request()
            request.name = self.reference_entity if identity == '__reference__' else self.registry[identity]['model_name']
            request.reference_frame = 'world' if identity == '__reference__' else self.reference_entity
            self.pending[identity] = Pending(self.get_client.call_async(request), time.monotonic())

    def _collect_states(self):
        for identity, pending in list(self.pending.items()):
            if time.monotonic()-pending.started > self.config['backend_timeout_sec']:
                self.pending.pop(identity)
                pending.future.cancel()
                if identity == '__reference__':
                    self.last_error = 'Reference state or timestamped TF timed out'
                continue
            if not pending.future.done():
                continue
            try:
                response = pending.response or pending.future.result()
                pending.response = response
                if response is None or not response.success:
                    self.pending.pop(identity)
                    self.cache.pop(identity, None)
                    if identity == '__reference__':
                        self.last_error = 'Gazebo reference entity unavailable'
                    continue
                stamp = stamp_ns(response.header.stamp)
                if stamp <= 0 or not self._fresh(stamp):
                    self.pending.pop(identity)
                    continue
                transform = self._tf(stamp)
                if identity == '__reference__':
                    quaternion_yaw(response.state.pose.orientation)
                    self.reference_stamp, self.reference_wall = stamp, time.monotonic()
                    self.last_error = ''
                else:
                    p, q = response.state.pose.position, response.state.pose.orientation
                    pose = transform_pose(Pose3(p.x, p.y, p.z, quaternion_yaw(q)), transform)
                    if not all(math.isfinite(v) for v in (pose.x, pose.y, pose.z, pose.yaw)):
                        raise ValueError('Invalid object pose')
                    item = self.registry[identity]
                    observation = Obstacle(identity, item['label'], pose, tuple(item['size']),
                                           item['confidence'], stamp, self.frame)
                    old = self.cache.get(identity)
                    if old is None or old[0].stamp_ns < stamp:
                        self.cache[identity] = observation, time.monotonic()
                self.pending.pop(identity)
            except Exception as error:
                # TF can lag /clock by one callback. Keep the response and
                # retry its exact timestamp until bounded timeout, never latest.
                self.last_error = f'Entity observation unavailable: {error}'

    def _command(self, pose, relative=False):
        if self.set_pending is not None or not self.held_id:
            return False
        request = SetEntityState.Request()
        request.state.name = self.registry[self.held_id]['model_name']
        request.state.reference_frame = self.reference_entity
        try:
            request.state.pose = ros_pose(pose if relative else transform_pose(pose, self._tf(), True))
            self.set_pending = Pending(self.set_client.call_async(request), time.monotonic())
            return True
        except Exception as error:
            self._finish(False, 'COMMAND_TF_FAILED', str(error))
            return False

    def _collect_command(self):
        pending = self.set_pending
        if pending is None:
            return
        if time.monotonic()-pending.started > self.config['backend_timeout_sec']:
            pending.future.cancel()
            self.set_pending = None
            self._finish(False, 'SET_STATE_TIMEOUT', 'Attachment outcome requires inspection')
        elif pending.future.done():
            self.set_pending = None
            try:
                response = pending.future.result()
                if response is None or not response.success:
                    self._finish(False, 'SET_STATE_REJECTED')
            except Exception as error:
                self._finish(False, 'SET_STATE_FAILED', str(error))

    def _move(self, target, step):
        if self.set_pending is not None:
            return False
        self.tcp = interpolate(self.tcp, target, step)
        if not self.injected:
            self._command(self.tcp)
        return distance(self.tcp, target) <= 1e-6 and abs(
            math.remainder(self.tcp.yaw-target.yaw, 2*math.pi)) <= 1e-6

    def _verify(self, target, lift=False):
        observation = self.get_observation(self.target_id)
        if observation is None or observation.stamp_ns <= max(self.verify_after, self.last_verified_stamp):
            return False
        self.last_verified_stamp = observation.stamp_ns
        p = observation.pose
        if lift:
            valid = p.z-self.original.z >= self.config['lift_min_displacement'] and distance(
                p, target) <= self.config['follow_tolerance']
        else:
            valid = distance(p, target) <= self.config['restore_position_tolerance'] and abs(
                math.remainder(p.yaw-target.yaw, 2*math.pi)) <= self.config['restore_yaw_tolerance']
        self.verify_count = self.verify_count+1 if valid else 0
        return self.verify_count >= int(self.config['verification_samples'])

    def _operate(self, dt):
        op, elapsed = self.operation, time.monotonic()-self.step_at
        step = self.config['lift_speed'] * min(dt, self.config['backend_poll_sec']*2)
        if op in ('PREGRASP', 'OPEN', 'APPROACH', 'CLOSE') and elapsed < self.config['action_step_sec']:
            return
        if op == 'PREGRASP':
            self._change('OPEN')
            self.gripper_open = True
        elif op == 'OPEN':
            self._change('APPROACH')
            self.tcp = self.original
        elif op == 'APPROACH':
            self.gripper_open = False
            self._change('CLOSE')
        elif op == 'CLOSE':
            if not self.injected:
                self.held_id = self.target_id
            self._change('LIFT')
        elif op == 'LIFT' and self._move(self.goal, step):
            self.verify_after = self._now()
            self._change('VERIFY_GRASP')
        elif op == 'VERIFY_GRASP':
            if self._verify(self.goal, lift=True):
                self.injected = False
                self._change('MOVE_TO_CARRY')
            elif elapsed > self.config['backend_timeout_sec']:
                if not self.held_id:
                    self._change('RETRACT_FAILED_GRASP')
                else:
                    self.cancel()
                    self.result = dict(success=False, code='GRASP_NOT_FOLLOWING',
                                       detail='No fresh consecutive target-following observations; attachment retained')
        elif op == 'RETRACT_FAILED_GRASP':
            target = Pose3(self.original.x, self.original.y,
                           self.original.z+self.config['pregrasp_offset'], self.original.yaw)
            self.tcp = interpolate(self.tcp, target, step)
            if distance(self.tcp, target) <= 1e-6:
                self.gripper_open = True
                self._finish(False, 'GRASP_NOT_FOLLOWING', 'No target motion; withdrew virtual gripper')
        elif op == 'MOVE_TO_CARRY':
            relative = Pose3(self.config['carry_x'], self.config['carry_y'],
                              self.config['carry_z'], 0.0)
            if self._move(transform_pose(relative, self._tf()), step):
                self.held_relative = relative
                self.verify_after = self._now()
                self.verify_count = 0
                self._change('VERIFY_CARRY')
        elif op == 'VERIFY_CARRY':
            self._command(self.held_relative, relative=True)
            if self._verify(transform_pose(self.held_relative, self._tf()), lift=True):
                self._finish(True, 'GRASP_VERIFIED', 'Virtual grasp and carry pose verified by entity observations')
            elif elapsed > self.config['backend_timeout_sec']:
                self._finish(False, 'CARRY_NOT_VERIFIED')
        elif op == 'PLACE_APPROACH':
            above = Pose3(self.goal.x, self.goal.y, self.goal.z+self.config['lift_height'], self.goal.yaw)
            if self._move(above, step):
                self._change('LOWER')
        elif op == 'LOWER' and self._move(self.goal, step):
            self._change('RELEASE')
        elif op == 'RELEASE' and self.set_pending is None:
            self.gripper_open = True
            self.held_id = self.held_relative = None
            self.verify_after = self._now()
            self._change('VERIFY_PLACE')
        elif op == 'VERIFY_PLACE':
            if self._verify(self.goal):
                self._finish(True, 'RESTORE_VERIFIED' if self.restoring else 'PLACE_VERIFIED')
            elif elapsed > self.config['backend_timeout_sec']:
                self._finish(False, 'RESTORE_NOT_VERIFIED' if self.restoring else 'PLACE_NOT_VERIFIED')

    def _markers(self):
        if self.tcp is None:
            return
        marker = Marker()
        marker.header.frame_id, marker.header.stamp = self.frame, self.node.get_clock().now().to_msg()
        marker.ns, marker.id, marker.type, marker.action = 'virtual_tcp', 0, Marker.SPHERE, Marker.ADD
        marker.pose = ros_pose(self.tcp)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.2, 0.8, 1.0, 0.9
        label = Marker()
        label.header, label.ns, label.id = marker.header, 'virtual_tcp', 1
        label.type, label.action, label.pose = Marker.TEXT_VIEW_FACING, Marker.ADD, ros_pose(
            Pose3(self.tcp.x, self.tcp.y, self.tcp.z+0.16, self.tcp.yaw))
        label.scale.z, label.color.r, label.color.g, label.color.b, label.color.a = 0.09, 1., 1., 1., 1.
        label.text = f'SIMULATED gripper: {self.operation} ({"open" if self.gripper_open else "closed"})'
        self.marker_pub.publish(MarkerArray(markers=[marker, label]))

    def tick(self):
        now, ros_now = time.monotonic(), self._now()
        dt, self.last_tick = now-self.last_tick, now
        if self.last_ros is None or ros_now > self.last_ros:
            self.clock_progress_at = now
        elif ros_now < self.last_ros:
            self.cache.clear()
            self.reference_stamp = 0
            if self.operation != 'IDLE':
                self.cancel()
                self.result = dict(success=False, code='CLOCK_RESET', detail='Fresh scene required')
        self.last_ros = ros_now
        self._collect_states()
        self._collect_command()
        if self.get_client.service_is_ready() and now-self.last_poll >= self.config['backend_poll_sec']:
            self.last_poll = now
            self._request_states()
        available, reason = self.ready()
        if self.operation != 'IDLE':
            if not available:
                self.cancel()
                self.result = dict(success=False, code='BACKEND_LOST', detail=reason)
            else:
                try:
                    self._operate(dt)
                except Exception as error:
                    self.cancel()
                    self.result = dict(success=False, code='OPERATION_FAILED', detail=str(error))
        elif self.held_id and self.held_relative is not None and available:
            self._command(self.held_relative, relative=True)
            try:
                self.tcp = transform_pose(self.held_relative, self._tf())
            except Exception:
                pass
        if now-self.last_marker >= self.config['backend_poll_sec']:
            self.last_marker = now
            self._markers()

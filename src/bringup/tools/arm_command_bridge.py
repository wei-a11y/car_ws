#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Accept simple arm test goals and serialize collision-checked MoveIt execution."""

import copy
import json
import math
import signal
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetMotionPlan
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String


def pose_constraints(target, link, position_tolerance, orientation_tolerance, position_only):
    """Build a spherical position goal and an optional orientation goal."""
    constraints = Constraints(name='arm_pose_goal')
    position = PositionConstraint()
    position.header = copy.deepcopy(target.header)
    position.link_name = link
    position.weight = 1.0
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tolerance])
    region_pose = Pose()
    region_pose.position = copy.deepcopy(target.pose.position)
    region_pose.orientation.w = 1.0
    position.constraint_region.primitives = [sphere]
    position.constraint_region.primitive_poses = [region_pose]
    constraints.position_constraints = [position]
    if not position_only:
        orientation = OrientationConstraint()
        orientation.header = copy.deepcopy(target.header)
        orientation.link_name = link
        orientation.orientation = copy.deepcopy(target.pose.orientation)
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0
        constraints.orientation_constraints = [orientation]
    return constraints


def joint_constraints(names, positions, tolerance):
    """Build bounded joint goals for both independent gripper fingers."""
    constraints = Constraints(name='gripper_command')
    for name, position in zip(names, positions):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=position, tolerance_above=tolerance,
            tolerance_below=tolerance, weight=1.0))
    return constraints


class ArmCommandBridge(Node):
    """Keep one planning/execution job active, rejecting overlapping commands."""

    def __init__(self):
        """Create parameterized subscriptions and asynchronous MoveIt clients."""
        super().__init__('arm_command_bridge')
        defaults = {
            'pose_topic': '/arm/pose_goal', 'gripper_topic': '/arm/gripper_command',
            'status_topic': '/arm/command_status', 'joint_states_topic': '/joint_states',
            'clock_topic': '/clock', 'planning_service': '/plan_kinematic_path',
            'execution_action': '/execute_trajectory', 'arm_group': 'arm',
            'gripper_group': 'gripper', 'target_frame': 'base_footprint',
            'end_effector_link': 'arm_5_link', 'position_only': True,
            'arm_joints': [f'arm_{i}_joint' for i in range(1, 6)],
            'arm_lower_bounds': [-1.55, 0.0, -3.14, -1.8, -3.14],
            'arm_upper_bounds': [1.55, 3.14, 0.04, 1.8, 3.14],
            'gripper_joints': ['clamp_left_joint', 'clamp_right_joint'],
            'gripper_lower_bounds': [-0.035, -0.035],
            'gripper_upper_bounds': [0.0, 0.0],
            'gripper_positions.open': [-0.025, -0.025],
            'gripper_positions.half': [-0.0125, -0.0125],
            'gripper_positions.close': [0.0, 0.0],
            'stationary_joints': ['arm_base_joint', 'L_wheel_joint', 'R_wheel_joint'],
            'stationary_velocity_limits': [0.005, 0.05, 0.05],
            'stationary_start_tolerances': [0.001, 0.05, 0.05],
            'state_timeout': 1.0, 'clock_timeout': 1.0, 'goal_timeout': 5.0,
            'future_stamp_tolerance': 0.05, 'planning_time': 5.0,
            'planning_attempts': 3, 'planning_response_timeout': 15.0,
            'execution_accept_timeout': 5.0, 'execution_time_margin': 10.0,
            'max_trajectory_duration': 30.0, 'feedback_timeout': 5.0,
            'position_tolerance': 0.001, 'orientation_tolerance': 0.01,
            'joint_goal_tolerance': 0.0001, 'arm_feedback_tolerance': 0.001,
            'gripper_feedback_tolerance': 0.0005, 'arm_start_tolerance': 0.02,
            'gripper_start_tolerance': 0.001, 'velocity_scaling': 0.1,
            'acceleration_scaling': 0.1, 'joint_limit_tolerance': 0.000001,
        }
        self.settings = {}
        for name, value in defaults.items():
            self.settings[name] = self.declare_parameter(
                name, value, descriptor=ParameterDescriptor(read_only=True)).value
        self._validate_settings(defaults)
        self.job = None
        self.sequence = 0
        self.last_state = None
        self.state_received_at = 0.0
        self.clock_ns = None
        self.clock_received_at = 0.0
        self.clock_progress_at = 0.0
        self.status = self.create_publisher(String, self.settings['status_topic'], 10)
        self.create_subscription(JointState, self.settings['joint_states_topic'],
                                 self._on_state, qos_profile_sensor_data)
        self.create_subscription(ClockMessage, self.settings['clock_topic'],
                                 self._on_clock, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self.settings['pose_topic'], self._on_pose, 1)
        self.create_subscription(String, self.settings['gripper_topic'], self._on_gripper, 1)
        self.planner = self.create_client(GetMotionPlan, self.settings['planning_service'])
        self.execution_client = ActionClient(
            self, ExecuteTrajectory, self.settings['execution_action'])
        self.create_timer(0.1, self._watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._report('IDLE', position_only=self.settings['position_only'],
                     frame=self.settings['target_frame'], link=self.settings['end_effector_link'])

    def _validate_settings(self, defaults):
        """Reject ambiguous names, incompatible arrays, and unsafe numeric settings."""
        for name, default in defaults.items():
            value = self.settings[name]
            if isinstance(default, str) and (not isinstance(value, str) or not value.strip()):
                raise ValueError(f'{name} must be a nonempty string')
            if isinstance(default, float) and (not math.isfinite(value) or value <= 0):
                raise ValueError(f'{name} must be finite and positive')
        for group in ('arm', 'gripper'):
            names = self.settings[f'{group}_joints']
            lower = self.settings[f'{group}_lower_bounds']
            upper = self.settings[f'{group}_upper_bounds']
            if not names or len(set(names)) != len(names) or not all(names):
                raise ValueError(f'{group}_joints must contain distinct nonempty names')
            if len(lower) != len(names) or len(upper) != len(names) or not all(
                    math.isfinite(a) and math.isfinite(b) and a <= b
                    for a, b in zip(lower, upper)):
                raise ValueError(
                    f'{group} bounds must match its joints and be finite ordered bounds')
        names = self.settings['stationary_joints']
        if not names or len(set(names)) != len(names) or not all(names):
            raise ValueError('stationary_joints must contain distinct nonempty names')
        for field in ('stationary_velocity_limits', 'stationary_start_tolerances'):
            values = self.settings[field]
            if len(values) != len(names) or not all(math.isfinite(v) and v > 0 for v in values):
                raise ValueError(f'{field} must contain one positive finite value per joint')
        all_names = self.settings['arm_joints'] + self.settings['gripper_joints'] + names
        if len(set(all_names)) != len(all_names):
            raise ValueError('Arm, gripper, and stationary joint lists must be disjoint')
        for command in ('open', 'half', 'close'):
            values = self.settings[f'gripper_positions.{command}']
            if len(values) != len(self.settings['gripper_joints']) or not all(
                    math.isfinite(value) and lower <= value <= upper
                    for value, lower, upper in zip(values, self.settings['gripper_lower_bounds'],
                                                   self.settings['gripper_upper_bounds'])):
                raise ValueError(f'gripper_positions.{command} violates configured bounds')
        if self.settings['planning_attempts'] <= 0:
            raise ValueError('planning_attempts must be positive')
        if any(self.settings[name] > 1.0 for name in ('velocity_scaling', 'acceleration_scaling')):
            raise ValueError('Velocity and acceleration scaling must be in (0, 1]')

    def _report(self, state, **fields):
        """Publish parseable job status and the same details to the ROS log."""
        job = self.job
        payload = {'state': state, 'job_id': job['id'] if job else 0,
                   'kind': job['kind'] if job else '', **fields}
        message = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status.publish(String(data=message))
        self.get_logger().info(message)

    def _on_state(self, message):
        """Store complete finite positions and velocities, clearing invalid feedback."""
        required = set(self.settings['arm_joints'] + self.settings['gripper_joints']
                       + self.settings['stationary_joints'])
        if len(set(message.name)) != len(message.name) or not required.issubset(message.name) or (
                len(message.position) != len(message.name)
                or len(message.velocity) != len(message.name)) or not all(
                math.isfinite(value) for value in list(message.position) + list(message.velocity)):
            self.last_state = None
            return
        self.last_state = copy.deepcopy(message)
        self.state_received_at = time.monotonic()

    def _on_clock(self, message):
        """Track progressing simulation time and invalidate feedback after clock reset."""
        ns = message.clock.sec * 10**9 + message.clock.nanosec
        now = time.monotonic()
        if self.clock_ns is not None and ns < self.clock_ns:
            self.last_state = None
            self.clock_progress_at = 0.0
            if self.job:
                self._abort('Simulation clock moved backwards')
        if self.clock_ns is None or ns > self.clock_ns:
            self.clock_progress_at = now
        self.clock_ns = ns
        self.clock_received_at = now

    def _current(self):
        """Require a usable clock, fresh full state, and a stationary base and slide."""
        wall_now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        if self.get_parameter('use_sim_time').value and (
                self.clock_ns is None or self.clock_ns <= 0
                or wall_now - self.clock_received_at > self.settings['clock_timeout']
                or wall_now - self.clock_progress_at > self.settings['clock_timeout']):
            raise ValueError('Simulation clock is missing, stale, or not advancing')
        if ros_now <= 0 or self.last_state is None or (
                wall_now - self.state_received_at > self.settings['state_timeout']):
            raise ValueError('Complete finite joint feedback is missing or stale')
        stamp = self.last_state.header.stamp
        age = (ros_now - stamp.sec * 10**9 - stamp.nanosec) / 1e9
        if stamp.sec == 0 and stamp.nanosec == 0 or not (
                -self.settings['future_stamp_tolerance'] <= age <= self.settings['state_timeout']):
            raise ValueError('Joint-state timestamp is stale or belongs to a different clock')
        velocities = dict(zip(self.last_state.name, self.last_state.velocity))
        positions = dict(zip(self.last_state.name, self.last_state.position))
        for kind in ('arm', 'gripper'):
            tolerance = self.settings[f'{kind}_feedback_tolerance']
            for name, lower, upper in zip(self.settings[f'{kind}_joints'],
                                          self.settings[f'{kind}_lower_bounds'],
                                          self.settings[f'{kind}_upper_bounds']):
                if not lower - tolerance <= positions[name] <= upper + tolerance:
                    raise ValueError(f'{name} feedback is outside configured position bounds')
        for name, limit in zip(self.settings['stationary_joints'],
                               self.settings['stationary_velocity_limits']):
            if abs(velocities[name]) > limit:
                raise ValueError(f'{name} is moving; wait for the base and slide to stop')
        return copy.deepcopy(self.last_state)

    def _on_pose(self, message):
        """Validate an end-effector target and request a MoveIt arm plan."""
        if self.job:
            self._report('REJECTED_BUSY', rejected_kind='arm')
            return
        try:
            target = copy.deepcopy(message)
            if target.header.frame_id != self.settings['target_frame']:
                raise ValueError(f'Expected pose frame {self.settings["target_frame"]}')
            point = target.pose.position
            if not all(math.isfinite(v) for v in (point.x, point.y, point.z)):
                raise ValueError('Target position must be finite')
            stamp = target.header.stamp
            if stamp.sec != 0 or stamp.nanosec != 0:
                age = (self.get_clock().now().nanoseconds
                       - stamp.sec * 10**9 - stamp.nanosec) / 1e9
                if not (-self.settings['future_stamp_tolerance'] <= age
                        <= self.settings['goal_timeout']):
                    raise ValueError('Pose goal timestamp is stale or in the future')
            if not self.settings['position_only']:
                q = target.pose.orientation
                values = [q.x, q.y, q.z, q.w]
                norm = math.hypot(*values)
                if not all(math.isfinite(v) for v in values) or (
                        not math.isfinite(norm) or norm < 1e-9):
                    raise ValueError('Full-pose mode requires a finite nonzero quaternion')
                q.x, q.y, q.z, q.w = [v / norm for v in values]
            # Resolve unstamped commands at acceptance, never at a later queued time.
            target.header.stamp = self.get_clock().now().to_msg()
            constraints = pose_constraints(
                target, self.settings['end_effector_link'], self.settings['position_tolerance'],
                self.settings['orientation_tolerance'], self.settings['position_only'])
            self._start('arm', constraints)
        except (ValueError, RuntimeError) as error:
            self._report('REJECTED', rejected_kind='arm', reason=str(error))

    def _on_gripper(self, message):
        """Accept exactly open, half, or close and plan the two finger joints."""
        if self.job:
            self._report('REJECTED_BUSY', rejected_kind='gripper')
            return
        try:
            if message.data not in ('open', 'half', 'close'):
                raise ValueError('Gripper command must be exactly open, half, or close')
            self._start('gripper', joint_constraints(
                self.settings['gripper_joints'],
                self.settings[f'gripper_positions.{message.data}'],
                self.settings['joint_goal_tolerance']))
        except (ValueError, RuntimeError) as error:
            self._report('REJECTED', rejected_kind='gripper', reason=str(error))

    def _start(self, kind, constraints):
        """Capture fresh state and start one bounded asynchronous planning request."""
        state = self._current()
        if not self.planner.service_is_ready() or not self.execution_client.server_is_ready():
            raise ValueError('MoveIt planning service or execution action is not ready')
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = self.settings[f'{kind}_group']
        motion.num_planning_attempts = self.settings['planning_attempts']
        motion.allowed_planning_time = self.settings['planning_time']
        motion.max_velocity_scaling_factor = self.settings['velocity_scaling']
        motion.max_acceleration_scaling_factor = self.settings['acceleration_scaling']
        motion.start_state.joint_state = state
        motion.start_state.is_diff = False
        motion.goal_constraints = [constraints]
        self.sequence += 1
        self.job = {'id': self.sequence, 'kind': kind, 'phase': 'PLANNING', 'start': state,
                    'handle': None, 'failure': None,
                    'deadline': time.monotonic() + self.settings['planning_response_timeout']}
        self._report('PLANNING', group=motion.group_name)
        identifier = self.sequence
        try:
            self.planner.call_async(request).add_done_callback(
                lambda future: self._planned(identifier, future))
        except Exception:
            self.job = None
            raise

    def _planned(self, identifier, future):
        """Validate the returned trajectory and start execution only from unchanged state."""
        if not self.job or self.job['id'] != identifier or self.job['phase'] != 'PLANNING':
            return
        try:
            response = future.result().motion_plan_response
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                raise ValueError(f'MoveIt planning failed with error {response.error_code.val}')
            trajectory = response.trajectory
            joint_trajectory = trajectory.joint_trajectory
            kind = self.job['kind']
            expected = self.settings[f'{kind}_joints']
            if set(joint_trajectory.joint_names) != set(expected) or (
                    len(joint_trajectory.joint_names) != len(expected)
                    or trajectory.multi_dof_joint_trajectory.joint_names
                    or not joint_trajectory.points):
                raise ValueError('Planner returned an empty trajectory or unexpected joints')
            bounds = dict(zip(expected, zip(
                self.settings[f'{kind}_lower_bounds'], self.settings[f'{kind}_upper_bounds'])))
            previous = -1.0
            for point in joint_trajectory.points:
                duration = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                if not math.isfinite(duration) or not 0 <= duration <= self.settings[
                        'max_trajectory_duration'] or duration <= previous:
                    raise ValueError('Trajectory timestamps are invalid or exceed duration limit')
                previous = duration
                tolerance = self.settings['joint_limit_tolerance']
                if len(point.positions) != len(expected) or not all(
                        math.isfinite(value)
                        and bounds[name][0] - tolerance <= value <= bounds[name][1] + tolerance
                        for name, value in zip(joint_trajectory.joint_names, point.positions)):
                    raise ValueError('Trajectory positions violate configured joint bounds')
                for values in (point.velocities, point.accelerations, point.effort):
                    if values and (len(values) != len(expected) or not all(
                            math.isfinite(value) for value in values)):
                        raise ValueError('Trajectory contains malformed or nonfinite joint data')
            current_state = self._current()
            current = dict(zip(current_state.name, current_state.position))
            start = dict(zip(self.job['start'].name, self.job['start'].position))
            tolerances = {name: self.settings['arm_start_tolerance']
                          for name in self.settings['arm_joints']}
            tolerances.update({name: self.settings['gripper_start_tolerance']
                               for name in self.settings['gripper_joints']})
            tolerances.update(zip(self.settings['stationary_joints'],
                                  self.settings['stationary_start_tolerances']))
            if any(abs(current[name] - start[name]) > tolerance
                   for name, tolerance in tolerances.items()):
                raise ValueError('Robot moved after planning; send a new goal from fresh state')
            self.job.update(phase='ACCEPTING', deadline=time.monotonic()
                            + self.settings['execution_accept_timeout'],
                            endpoint=dict(zip(joint_trajectory.joint_names,
                                              joint_trajectory.points[-1].positions)),
                            duration=previous)
            goal = ExecuteTrajectory.Goal(trajectory=trajectory)
            self.execution_client.send_goal_async(goal).add_done_callback(
                lambda pending: self._accepted(identifier, pending))
            self._report('EXECUTING', planned_duration=previous)
        except Exception as error:
            self._finish(False, f'{type(error).__name__}: {error}')

    def _accepted(self, identifier, future):
        """Record the execution handle and wait for a terminal action result."""
        if not self.job or self.job['id'] != identifier:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                self._finish(False, 'MoveIt execution goal was rejected')
                return
            self.job['handle'] = handle
            self.job['phase'] = 'EXECUTING'
            self.job['deadline'] = time.monotonic() + self.job['duration'] + self.settings[
                'execution_time_margin']
            handle.get_result_async().add_done_callback(
                lambda pending: self._executed(identifier, pending))
            if self.job['failure']:
                self._abort(self.job['failure'])
        except Exception as error:
            self._abort(f'Execution acceptance failed: {error}')

    def _executed(self, identifier, future):
        """Require action success and then verify fresh actual joint convergence."""
        if not self.job or self.job['id'] != identifier:
            return
        try:
            result = future.result()
            code = result.result.error_code.val
            if self.job['failure'] or result.status != GoalStatus.STATUS_SUCCEEDED or (
                    code != MoveItErrorCodes.SUCCESS):
                self._finish(False, self.job['failure'] or (
                    f'Execution failed: action status {result.status}, MoveIt error {code}'))
                return
            self.job.update(phase='VERIFYING', deadline=time.monotonic()
                            + self.settings['feedback_timeout'])
            self._report('VERIFYING')
        except Exception as error:
            self._abort(f'Execution result failed: {error}')

    def _abort(self, reason):
        """Cancel possible execution and hold busy until its terminal result is known."""
        if not self.job:
            return
        if self.job['phase'] in ('PLANNING', 'VERIFYING'):
            self._finish(False, reason)
            return
        if self.job['failure'] is None:
            self._report('FAILED', reason=reason, busy_until_execution_stops=True)
        self.job['failure'] = reason
        self.job['phase'] = 'STOPPING'
        self.job['deadline'] = None
        handle = self.job['handle']
        if handle is not None:
            handle.cancel_goal_async()

    def _finish(self, success, reason=''):
        """Publish a terminal job result before allowing another command."""
        self._report('SUCCEEDED' if success else 'FAILED', reason=reason,
                     endpoint=self.job.get('endpoint', {}) if self.job else {})
        self.job = None

    def _watchdog(self):
        """Use wall time for deadlines even if simulation time pauses."""
        if not self.job:
            return
        if self.job['deadline'] is not None and time.monotonic() > self.job['deadline']:
            self._abort(f'{self.job["phase"]} timed out')
            return
        if self.job['phase'] == 'STOPPING':
            return
        try:
            state = self._current()
        except ValueError as error:
            self._abort(str(error))
            return
        if self.job['phase'] == 'VERIFYING':
            positions = dict(zip(state.name, state.position))
            tolerance = self.settings[f'{self.job["kind"]}_feedback_tolerance']
            if all(abs(positions[name] - value) <= tolerance
                   for name, value in self.job['endpoint'].items()):
                self._finish(True)

    def stop_execution(self):
        """Request action cancellation during orderly node shutdown."""
        if self.job and self.job['phase'] not in ('PLANNING', 'VERIFYING'):
            self._abort('Command bridge shutting down')


def main(args=None):
    """Run the optional command bridge against an already running MoveIt stack."""
    # Keep the ROS context alive long enough to cancel a goal on SIGINT/SIGTERM.
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ArmCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_execution()
            deadline = time.monotonic() + 2.0
            while node.job and node.job['phase'] != 'PLANNING' and (
                    rclpy.ok() and time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.job and node.job['phase'] != 'PLANNING':
                node.get_logger().warning(
                    'Execution stop was not confirmed before shutdown; '
                    'inspect MoveIt/controller state')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

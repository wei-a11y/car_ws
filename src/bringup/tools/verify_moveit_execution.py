#!/usr/bin/env python3
# Copyright 2026 car_ws contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Verify MoveIt planning, execution, and joint feedback in mock or Gazebo.

Run against a fresh simulation launch in a dedicated nonzero ROS_DOMAIN_ID.
This tool executes small arm and gripper trajectories, checks actual feedback,
and never publishes base commands. Gazebo's starting joint positions must
match the initial values in its runtime robot_description. Reusing a simulation
after moving the arm requires a fresh launch before this startup verification.
"""

import argparse
import copy
import json
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


ARM_JOINTS = [f"arm_{i}_joint" for i in range(1, 6)]
GRIPPER_JOINTS = ["clamp_left_joint", "clamp_right_joint"]
MANIPULATOR_JOINTS = ARM_JOINTS + ["arm_base_joint"] + GRIPPER_JOINTS
UNCHANGED_JOINTS = ["arm_base_joint", "L_wheel_joint", "R_wheel_joint"]
ALL_JOINTS = set(ARM_JOINTS + GRIPPER_JOINTS + UNCHANGED_JOINTS)
CONTROLLERS = {"arm_controller", "gripper_controller", "joint_state_broadcaster"}


def emit(event, **fields):
    """Emit a machine-readable verification event."""
    print(json.dumps({"event": event, **fields}, sort_keys=True), flush=True)


class MoveItExecutionCheck(Node):
    """Check an isolated simulation and execute bounded MoveIt test trajectories."""

    def __init__(self, args):
        """Create clients and feedback subscriptions for the selected simulation mode."""
        self.mode = args.mode
        self.args = args
        super().__init__(
            "car_ws_moveit_execution_check",
            parameter_overrides=[Parameter("use_sim_time", value=self.mode == "gazebo")],
        )
        self.last_state = None
        self.last_state_at = 0.0
        self.last_clock_ns = None
        self.last_clock_at = 0.0
        self.last_odom = None
        self.last_odom_at = 0.0
        self.runtime_robot = None
        self.hold_samples = None
        self.motion_samples = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(
            JointState, "/joint_states", self.on_state, qos_profile_sensor_data
        )
        self.controllers = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.hardware = self.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components"
        )
        self.planner = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.execution = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")

    def on_clock(self, msg):
        """Record the latest simulation clock sample."""
        self.last_clock_ns = msg.clock.sec * 10**9 + msg.clock.nanosec
        self.last_clock_at = time.monotonic()

    def on_odom(self, msg):
        """Record finite odometry with the configured base frames."""
        pose = msg.pose.pose
        values = [pose.position.x, pose.position.y, pose.position.z,
                  pose.orientation.x, pose.orientation.y,
                  pose.orientation.z, pose.orientation.w,
                  msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        if msg.header.frame_id == "odom" and msg.child_frame_id == "base_footprint" and all(
            math.isfinite(value) for value in values
        ):
            self.last_odom = msg
            self.last_odom_at = time.monotonic()

    def assert_single_manager(self):
        """Require exactly one root controller manager after bounded discovery."""
        self.spin_until(
            lambda: any(name == "controller_manager"
                        for name, _ in self.get_node_names_and_namespaces()),
            5.0, "controller_manager graph discovery",
        )
        managers = [
            (name, namespace) for name, namespace in self.get_node_names_and_namespaces()
            if name == "controller_manager"
        ]
        if managers != [("controller_manager", "/")]:
            raise RuntimeError(f"expected single /controller_manager, found {managers}")
        emit("single_controller_manager", nodes=managers)

    def check_node_clocks(self, controller_names):
        """Verify simulation-time settings for all required nodes and controllers."""
        mandatory = {
            "controller_manager", "robot_state_publisher", "move_group"
        } | set(controller_names)
        self.spin_until(
            lambda: mandatory.issubset(
                {name for name, _ in self.get_node_names_and_namespaces()}
            ),
            5.0, "required node graph discovery for clock checks",
        )
        nodes = self.get_node_names_and_namespaces()
        wanted = mandatory | set(controller_names) | {"rviz2", "rviz"}
        found = {name for name, _ in nodes}
        if not mandatory.issubset(found):
            raise RuntimeError(f"missing ROS nodes: {sorted(mandatory - found)}")
        clocks = {}
        clients = []
        for name, namespace in nodes:
            if name not in wanted:
                continue
            full_name = f"{namespace.rstrip('/')}/{name}"
            client = self.create_client(GetParameters, f"{full_name}/get_parameters")
            clients.append(client)
            self.spin_until(client.service_is_ready, 3.0, f"{full_name} parameter service")
            request = GetParameters.Request()
            request.names = ["use_sim_time"]
            response = self.result(client.call_async(request), 3.0, f"{full_name} use_sim_time")
            if not response.values or response.values[0].type != ParameterType.PARAMETER_BOOL:
                raise RuntimeError(f"{full_name} has no boolean use_sim_time")
            clocks[full_name] = response.values[0].bool_value
        if any(value != (self.mode == "gazebo") for value in clocks.values()):
            raise RuntimeError(f"unexpected use_sim_time in {self.mode} mode: {clocks}")
        emit("node_clocks", use_sim_time=clocks)
        for client in clients:
            self.destroy_client(client)

    def check_gazebo_inputs(self):
        """Require advancing simulation time, fresh odometry, and the base TF edge."""
        self.spin_until(
            lambda: self.last_clock_ns is not None and self.last_clock_ns > 0,
            10.0, "nonzero simulation clock",
        )
        first_clock = self.last_clock_ns
        self.spin_until(
            lambda: self.last_clock_ns > first_clock + 100_000_000,
            10.0, "advancing simulation clock",
        )
        self.spin_until(
            lambda: self.last_odom is not None and time.monotonic() - self.last_odom_at < 1.0,
            10.0, "fresh odometry",
        )
        self.spin_until(
            lambda: self.tf_buffer.can_transform("odom", "base_footprint", Time()),
            10.0, "odom -> base_footprint TF",
        )
        transform = self.tf_buffer.lookup_transform("odom", "base_footprint", Time())
        emit("gazebo_inputs", clock_advance_s=(self.last_clock_ns - first_clock) / 1e9,
             odom_frames=[self.last_odom.header.frame_id, self.last_odom.child_frame_id],
             tf_frames=[transform.header.frame_id, transform.child_frame_id])

    def get_runtime_robot(self):
        """Read and cache the actual robot description published by RSP."""
        if self.runtime_robot is not None:
            return self.runtime_robot
        self.spin_until(
            lambda: ("robot_state_publisher", "/") in self.get_node_names_and_namespaces(),
            5.0, "robot_state_publisher graph discovery for hardware verification",
        )
        client = self.create_client(
            GetParameters, "/robot_state_publisher/get_parameters"
        )
        try:
            self.spin_until(
                client.service_is_ready, 5.0, "RSP robot_description parameter service"
            )
            request = GetParameters.Request()
            request.names = ["robot_description"]
            response = self.result(
                client.call_async(request), 5.0, "RSP robot_description parameter"
            )
            if len(response.values) != 1 or (
                response.values[0].type != ParameterType.PARAMETER_STRING
            ):
                raise RuntimeError("RSP has no string robot_description")
            self.runtime_robot = ET.fromstring(response.values[0].string_value)
            return self.runtime_robot
        finally:
            self.destroy_client(client)

    def verify_gazebo_description_hardware(self, runtime_classes):
        """Verify Gazebo hardware when its runtime class metadata is unavailable."""
        systems = self.get_runtime_robot().findall("ros2_control")
        if len(systems) != 1:
            raise RuntimeError(
                f"Gazebo runtime description must contain exactly one ros2_control, "
                f"found {len(systems)}"
            )
        system = systems[0]
        system_name = system.get("name")
        if set(runtime_classes) != {system_name}:
            raise RuntimeError(
                f"Gazebo description hardware name {system_name!r} does not match "
                f"runtime components {sorted(runtime_classes)}"
            )
        plugins = system.findall("hardware/plugin")
        if len(plugins) != 1 or (plugins[0].text or "").strip() != (
            "gazebo_ros2_control/GazeboSystem"
        ):
            raise RuntimeError("Gazebo runtime description has unexpected hardware plugin")
        return {system_name: plugins[0].text.strip()}

    def check_gazebo_hold(self):
        """Require a bounded low-velocity hold near the declared initial positions."""
        robot = self.get_runtime_robot()
        specs = {}
        for name in MANIPULATOR_JOINTS:
            joint = robot.find(f"joint[@name='{name}']")
            limit = joint.find("limit") if joint is not None else None
            initial_text = robot.findtext(
                f"ros2_control/joint[@name='{name}']/state_interface[@name='position']"
                "/param[@name='initial_value']"
            )
            if limit is None or initial_text is None:
                raise RuntimeError(
                    f"runtime model missing {name} position bounds or initial_value"
                )
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
            initial = float(initial_text)
            if not all(math.isfinite(v) for v in [lower, upper, initial]) or not (
                lower <= initial <= upper
            ):
                raise RuntimeError(f"invalid runtime position bounds/initial value for {name}")
            linear = joint.get("type") == "prismatic"
            specs[name] = {
                "lower": lower, "upper": upper, "initial": initial,
                "bound_tolerance": (
                    self.args.linear_bound_tolerance
                    if linear else self.args.angular_bound_tolerance
                ),
                "velocity_tolerance": (
                    self.args.linear_hold_velocity
                    if linear else self.args.angular_hold_velocity
                ),
                "drift_tolerance": (
                    self.args.linear_hold_drift if linear else self.args.angular_hold_drift
                ),
                "initial_tolerance": (
                    self.args.linear_initial_tolerance
                    if linear else self.args.angular_initial_tolerance
                ),
            }
        emit("gazebo_hold_requirements", minimum_sim_duration_s=1.0,
             settle_timeout_s=self.args.settle_timeout, joints=specs)
        self.hold_samples = []
        candidate = []
        last_problem = "waiting for joint samples"
        deadline = time.monotonic() + self.args.settle_timeout
        try:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())))
                pending, self.hold_samples = self.hold_samples, []
                for sample in pending:
                    problem = None
                    for name, spec in specs.items():
                        position = sample["positions"].get(name)
                        velocity = sample["velocities"].get(name)
                        if position is None or velocity is None or not all(
                            math.isfinite(v) for v in [position, velocity]
                        ):
                            problem = f"{name} missing/nonfinite position or velocity"
                        elif not (
                            spec["lower"] - spec["bound_tolerance"] <= position
                            <= spec["upper"] + spec["bound_tolerance"]
                        ):
                            problem = (
                                f"{name} position {position} outside "
                                f"[{spec['lower']}, {spec['upper']}]"
                            )
                        elif abs(position - spec["initial"]) > spec["initial_tolerance"]:
                            problem = (
                                f"{name} position {position} differs from initial "
                                f"{spec['initial']}"
                            )
                        elif abs(velocity) > spec["velocity_tolerance"]:
                            problem = (
                                f"{name} velocity {velocity} exceeds hold threshold "
                                f"{spec['velocity_tolerance']}"
                            )
                        if problem:
                            break
                    if candidate and sample["stamp_ns"] < candidate[-1]["stamp_ns"]:
                        problem = "joint_states simulation timestamp moved backward"
                    if problem:
                        candidate = []
                        last_problem = problem
                        continue
                    candidate.append(sample)
                    for name, spec in specs.items():
                        values = [item["positions"][name] for item in candidate]
                        if max(values) - min(values) > spec["drift_tolerance"]:
                            last_problem = (
                                f"{name} holding drift exceeds {spec['drift_tolerance']}"
                            )
                            candidate = [sample]
                            break
                    duration_ns = candidate[-1]["stamp_ns"] - candidate[0]["stamp_ns"]
                    if duration_ns >= 1_000_000_000 and len(candidate) >= 10:
                        stats = {}
                        for name in specs:
                            values = [item["positions"][name] for item in candidate]
                            stats[name] = {
                                "initial": specs[name]["initial"],
                                "range": [min(values), max(values)],
                                "drift": max(values) - min(values),
                                "peak_abs_velocity": max(
                                    abs(item["velocities"][name]) for item in candidate
                                ),
                                "maximum_initial_offset": max(
                                    abs(v - specs[name]["initial"]) for v in values
                                ),
                            }
                        emit("gazebo_hold_stable", duration_s=duration_ns / 1e9,
                             samples=len(candidate), joints=stats)
                        return
            raise RuntimeError(
                f"no stable bounded Gazebo hold within {self.args.settle_timeout}s: {last_problem}"
            )
        finally:
            self.hold_samples = None

    def on_state(self, msg):
        """Collect physical joint feedback for readiness, holding, and execution."""
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if self.motion_samples is not None:
            self.motion_samples.append((values, dict(zip(msg.name, msg.velocity))))
        if self.hold_samples is not None:
            self.hold_samples.append({
                "stamp_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                "positions": values,
                "velocities": dict(zip(msg.name, msg.velocity)),
            })
        if ALL_JOINTS.issubset(values) and all(
            math.isfinite(values[name]) for name in ALL_JOINTS
        ):
            self.last_state = msg
            self.last_state_at = time.monotonic()

    def spin_until(self, predicate, timeout, label):
        """Process callbacks until a predicate holds or a wall-time deadline expires."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return
            rclpy.spin_once(self, timeout_sec=max(0.0, min(0.1, deadline - time.monotonic())))
        if predicate():
            return
        raise RuntimeError(f"timeout waiting for {label} ({timeout:.1f}s)")

    def result(self, future, timeout, label):
        """Wait for a nonempty service or action result within a bounded deadline."""
        self.spin_until(future.done, timeout, label)
        result = future.result()
        if result is None:
            raise RuntimeError(f"{label} returned no result")
        return result

    def current(self, timeout=5.0):
        """Return a copy of complete, finite, and fresh joint feedback."""
        self.spin_until(
            lambda: self.last_state is not None
            and time.monotonic() - self.last_state_at < 1.0,
            timeout,
            "complete fresh joint_states",
        )
        return copy.deepcopy(self.last_state)

    def ready(self, timeout):
        """Check controllers, hardware, clocks, feedback, and MoveIt readiness."""
        controller_names = set(CONTROLLERS)
        if self.mode == "gazebo":
            controller_names.add("diff_drive_controller")
            controller_names.add("arm_base_controller")
        deadline = time.monotonic() + timeout
        self.spin_until(self.controllers.service_is_ready, timeout, "controller manager")
        active = {}
        while time.monotonic() < deadline:
            response = self.result(
                self.controllers.call_async(ListControllers.Request()),
                min(3.0, max(0.1, deadline - time.monotonic())),
                "ListControllers",
            )
            active = {item.name: item.state for item in response.controller}
            if all(active.get(name) == "active" for name in controller_names):
                break
            tick = time.monotonic() + 0.2
            self.spin_until(lambda: time.monotonic() >= tick, 0.3, "controller retry tick")
        else:
            raise RuntimeError(f"controllers not active: {active}")
        emit("controllers", states=active)
        self.assert_single_manager()

        self.spin_until(self.hardware.service_is_ready, 5.0, "hardware listing")
        hardware = self.result(
            self.hardware.call_async(ListHardwareComponents.Request()),
            5.0,
            "ListHardwareComponents",
        ).component
        classes = {item.name: item.class_type for item in hardware}
        expected_plugin = ("gazebo_ros2_control/GazeboSystem" if self.mode == "gazebo"
                           else "mock_components/GenericSystem")
        description_plugins = None
        if self.mode == "gazebo" and classes and all(plugin == "" for plugin in classes.values()):
            description_plugins = self.verify_gazebo_description_hardware(classes)
        elif not classes or any(plugin != expected_plugin for plugin in classes.values()):
            raise RuntimeError(f"expected only {expected_plugin} hardware, found {classes}")
        emit("hardware", plugins=classes,
             runtime_class_metadata_empty=description_plugins is not None,
             description_plugins=description_plugins)
        self.check_node_clocks(controller_names)
        if self.mode == "gazebo":
            self.check_gazebo_inputs()
            self.check_gazebo_hold()

        base_publishers = {
            topic: len(self.get_publishers_info_by_topic(topic))
            for topic in ["/cmd_vel", "/cmd_vel_raw"]
        }
        if any(base_publishers.values()):
            raise RuntimeError(f"unexpected base command publishers: {base_publishers}")
        emit("base_command_publishers", counts=base_publishers)
        state = self.current()
        emit("joint_states", positions=dict(zip(state.name, state.position)))
        self.spin_until(self.planner.service_is_ready, 10.0, "MoveIt planning service")
        self.spin_until(self.execution.server_is_ready, 10.0, "MoveIt execution action")

    def plan_and_execute(self, group, target):
        """Plan and execute a joint goal, then verify actual feedback and base drift."""
        state = self.current()
        before = dict(zip(state.name, state.position))
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = group
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.1
        motion.max_acceleration_scaling_factor = 0.1
        motion.start_state.joint_state = state
        motion.start_state.is_diff = False
        constraint = Constraints()
        constraint.name = f"{group}_execution_check"
        for name, position in target.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = position
            joint.tolerance_above = 0.0001
            joint.tolerance_below = 0.0001
            joint.weight = 1.0
            constraint.joint_constraints.append(joint)
        motion.goal_constraints = [constraint]

        planned = self.result(
            self.planner.call_async(request), 15.0, f"{group} GetMotionPlan"
        ).motion_plan_response
        if planned.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{group} planning error {planned.error_code.val}")
        trajectory = planned.trajectory
        joint_trajectory = trajectory.joint_trajectory
        if set(joint_trajectory.joint_names) != set(target):
            raise RuntimeError(
                f"{group} unexpected planned joints: {joint_trajectory.joint_names}"
            )
        if trajectory.multi_dof_joint_trajectory.joint_names or not joint_trajectory.points:
            raise RuntimeError(f"{group} unexpected multi-DOF or empty trajectory")
        final = joint_trajectory.points[-1]
        planned_endpoint = dict(zip(joint_trajectory.joint_names, final.positions))
        if any(abs(planned_endpoint[name] - value) > 0.0002 for name, value in target.items()):
            raise RuntimeError(f"{group} planned endpoint differs from goal: {planned_endpoint}")
        duration = final.time_from_start.sec + final.time_from_start.nanosec / 1e9
        if not math.isfinite(duration) or duration > 30.0:
            raise RuntimeError(f"{group} unexpected trajectory duration {duration}")
        emit(
            "planned",
            group=group,
            error_code=planned.error_code.val,
            joints=list(joint_trajectory.joint_names),
            points=len(joint_trajectory.points),
            duration_s=round(duration, 4),
            target=target,
        )

        if self.mode == "gazebo":
            self.motion_samples = []
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle = self.result(
            self.execution.send_goal_async(goal), 5.0, f"{group} execution acceptance"
        )
        if not handle.accepted:
            raise RuntimeError(f"{group} MoveIt execution rejected")
        try:
            result = self.result(
                handle.get_result_async(), max(10.0, duration + 10.0), f"{group} execution"
            )
        except Exception:
            self.result(handle.cancel_goal_async(), 3.0, f"{group} execution cancellation")
            raise
        if result.status != GoalStatus.STATUS_SUCCEEDED or (
            result.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                f"{group} execution failed: status={result.status}, "
                f"MoveIt error={result.result.error_code.val}"
            )

        def converged():
            """Check whether fresh joint feedback matches every commanded joint."""
            if self.last_state is None or time.monotonic() - self.last_state_at >= 1.0:
                return False
            actual = dict(zip(self.last_state.name, self.last_state.position))
            return all(abs(actual[name] - value) <= 0.001 for name, value in target.items())

        self.spin_until(converged, 5.0, f"{group} joint feedback convergence")
        actual = dict(zip(self.current().name, self.current().position))
        drift = {name: actual[name] - before[name] for name in UNCHANGED_JOINTS}
        slide_tolerance = self.args.slide_tolerance if self.mode == "gazebo" else 1e-8
        wheel_tolerance = self.args.wheel_tolerance if self.mode == "gazebo" else 1e-8
        if abs(drift["arm_base_joint"]) > slide_tolerance or any(
            abs(drift[name]) > wheel_tolerance for name in ["L_wheel_joint", "R_wheel_joint"]
        ):
            raise RuntimeError(f"{group} slide/wheel drift beyond tolerance: {drift}")
        if self.mode == "gazebo":
            samples, self.motion_samples = self.motion_samples, None
            specs = {}
            for name in MANIPULATOR_JOINTS:
                joint = self.get_runtime_robot().find(f"joint[@name='{name}']")
                lim = joint.find("limit")
                linear = joint.get("type") == "prismatic"
                tol = (
                    self.args.linear_bound_tolerance
                    if linear else self.args.angular_bound_tolerance
                )
                specs[name] = (float(lim.get("lower")) - tol, float(lim.get("upper")) + tol,
                               float(lim.get("velocity")) * 1.1)
            if len(samples) < 10:
                raise RuntimeError("not enough observed trajectory feedback")
            peaks = {}
            for name, (lower, upper, vmax) in specs.items():
                for pos, vel in samples:
                    if name not in pos or name not in vel or not all(
                        math.isfinite(v) for v in [pos[name], vel[name]]
                    ):
                        raise RuntimeError(f"nonfinite/missing {name} during execution")
                    if not lower <= pos[name] <= upper or abs(vel[name]) > vmax:
                        raise RuntimeError(
                            f"{name} violated bounds/velocity during execution: "
                            f"{pos[name]}, {vel[name]}"
                        )
                peaks[name] = max(abs(vel[name]) for _, vel in samples)
            emit("execution_feedback_limits", group=group, samples=len(samples),
                 peak_abs_velocity=peaks)
        emit(
            "executed",
            group=group,
            action_status=result.status,
            error_code=result.result.error_code.val,
            actual={name: actual[name] for name in target},
            max_error=max(abs(actual[name] - value) for name, value in target.items()),
            unchanged={name: actual[name] for name in UNCHANGED_JOINTS},
            slide_wheel_drift=drift,
        )


def main():
    """Parse thresholds and verify an isolated mock or Gazebo simulation."""
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--mode", choices=["mock", "gazebo"], required=True,
                        help="Fresh simulation launch to verify")
    parser.add_argument("--startup-timeout", type=float, default=45.0,
                        help="Maximum wall seconds for controller readiness (at most 60)")
    parser.add_argument("--slide-tolerance", type=float, default=0.003,
                        help="Gazebo slide position drift tolerance in metres")
    parser.add_argument("--wheel-tolerance", type=float, default=0.05,
                        help="Gazebo wheel position drift tolerance in radians")
    parser.add_argument("--settle-timeout", type=float, default=15.0,
                        help="Maximum wall seconds to observe a stable Gazebo hold (at most 15)")
    parser.add_argument("--angular-bound-tolerance", type=float, default=0.001,
                        help="Numerical tolerance for angular position limits in radians")
    parser.add_argument("--linear-bound-tolerance", type=float, default=0.0001,
                        help="Numerical tolerance for linear position limits in metres")
    parser.add_argument("--angular-hold-velocity", type=float, default=0.05,
                        help="Maximum angular speed during Gazebo startup hold in radians/s")
    parser.add_argument("--linear-hold-velocity", type=float, default=0.005,
                        help="Maximum linear speed during Gazebo startup hold in metres/s")
    parser.add_argument("--angular-hold-drift", type=float, default=0.005,
                        help="Maximum angular position range during startup hold in radians")
    parser.add_argument("--linear-hold-drift", type=float, default=0.0005,
                        help="Maximum linear position range during startup hold in metres")
    parser.add_argument("--angular-initial-tolerance", type=float, default=0.05,
                        help="Maximum startup offset from initial angular positions in radians")
    parser.add_argument("--linear-initial-tolerance", type=float, default=0.003,
                        help="Maximum startup offset from initial linear positions in metres")
    args = parser.parse_args()
    if not math.isfinite(args.startup_timeout) or not 0 < args.startup_timeout <= 60:
        parser.error("--startup-timeout must be finite and in (0, 60]")
    if not all(math.isfinite(v) and v > 0 for v in [args.slide_tolerance, args.wheel_tolerance]):
        parser.error("drift tolerances must be finite and positive")
    if not math.isfinite(args.settle_timeout) or not 0 < args.settle_timeout <= 15:
        parser.error("--settle-timeout must be finite and in (0, 15]")
    for field in ["angular_bound_tolerance", "linear_bound_tolerance", "angular_hold_velocity",
                  "linear_hold_velocity", "angular_hold_drift", "linear_hold_drift",
                  "angular_initial_tolerance", "linear_initial_tolerance"]:
        if not math.isfinite(getattr(args, field)) or getattr(args, field) <= 0:
            parser.error(f"--{field.replace('_', '-')} must be finite and positive")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    try:
        domain_id = int(domain) if domain.isascii() and domain.isdecimal() else 0
    except ValueError:
        domain_id = 0
    if domain_id <= 0:
        emit("failed", reason="set an isolated positive integer ROS_DOMAIN_ID before running")
        return 1
    rclpy.init()
    node = MoveItExecutionCheck(args)
    try:
        emit("started", ros_domain_id=domain, mode=args.mode)
        node.ready(args.startup_timeout)
        arm_target = dict(zip(ARM_JOINTS, [0.1, 0.2, -0.3, 0.1, 0.1]))
        node.plan_and_execute("arm", arm_target)
        node.plan_and_execute("gripper", {name: -0.005 for name in GRIPPER_JOINTS})
        node.assert_single_manager()
        checks = ["single_controller_manager", "active_controllers", "expected_hardware",
                  "correct_use_sim_time", "full_joint_states", "arm_plan_execute_feedback",
                  "gripper_plan_execute_feedback", "stationary_slide_and_wheels"]
        if args.mode == "gazebo":
            checks.append("bounded_low_velocity_hold_at_runtime_initial_state_for_one_sim_second")
        emit("passed", mode=args.mode, checks=checks)
        return 0
    except Exception as exc:
        emit("failed", reason=f"{type(exc).__name__}: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())

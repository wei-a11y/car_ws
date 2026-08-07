"""High-level chassis action server for approach, search, side align, and return."""

import math
import threading
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from delivery_robot_interfaces.action import ExecuteChassisTask
from delivery_robot_interfaces.msg import (
    LocalizationState,
    ModuleState,
    StationTargetState,
)
from delivery_robot_interfaces.srv import SetLockerDoor
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from .side_alignment import (
    compute_side_goal,
    normalize_angle,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


class ChassisTaskServer(Node):
    """Own all chassis motion while exposing one business-level action."""

    ERR_NOT_LOCALIZED = 2001
    ERR_MODULE_UNSAFE = 2002
    ERR_NAV_SERVER = 2101
    ERR_PLAN_FAILED = 2102
    ERR_NAV_FAILED = 2103
    ERR_NAV_TIMEOUT = 2104
    ERR_TARGET_NOT_FOUND = 2201
    ERR_ALIGN_TIMEOUT = 2301
    ERR_TF = 2302
    ERR_INVALID_GOAL = 2401
    ERR_CANCELLED = 2402
    ERR_OBSTACLE = 2901

    def __init__(self) -> None:
        super().__init__("chassis_task_server")
        defaults = {
            "map_frame": "map",
            "base_frame": "base_footprint",
            "navigate_action": "/navigate_to_pose",
            "compute_path_action": "/compute_path_to_pose",
            "localization_required": True,
            "require_module_safety_states": True,
            "nav_server_timeout_s": 5.0,
            "default_action_timeout_s": 120.0,
            "planner_id": "GridBased",
            "docking_side": "right",
            "base_side_offset_m": 0.35,
            "side_reference_x_m": 0.0,
            "front_opening_center_xyz_m": [0.55, 0.0, 0.65],
            "front_opening_y_tolerance_m": 0.015,
            "front_opening_z_tolerance_m": 0.150,
            "target_gap_m": 0.075,
            "min_gap_m": 0.05,
            "max_gap_m": 0.10,
            "lateral_tolerance_m": 0.02,
            "xy_tolerance_m": 0.015,
            "yaw_tolerance_rad": 0.0349066,
            "stable_cycles": 10,
            "control_rate_hz": 20.0,
            "align_timeout_s": 30.0,
            "search_timeout_s": 20.0,
            "search_yaw_rate_rps": 0.10,
            "search_sweep_period_s": 2.0,
            "max_align_linear_mps": 0.05,
            "max_align_angular_rps": 0.20,
            "linear_gain": 0.8,
            "heading_gain": 1.5,
            "yaw_gain": 1.2,
            "rotate_in_place_threshold_rad": 0.35,
            "hard_stop_range_m": 0.12,
            "use_target_hint_in_simulation": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._callback_group = ReentrantCallbackGroup()
        self._navigate = ActionClient(
            self,
            NavigateToPose,
            str(self.get_parameter("navigate_action").value),
            callback_group=self._callback_group,
        )
        self._compute_path = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("compute_path_action").value),
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteChassisTask,
            "/chassis/execute_task",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        target_qos = QoSProfile(depth=5)
        target_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._module_pub = self.create_publisher(
            ModuleState, "/chassis/state", latched
        )
        self._target_pub = self.create_publisher(
            StationTargetState, "/chassis/station_target_state", target_qos
        )
        self._align_pub = self.create_publisher(Twist, "/cmd_vel_align", 10)
        self._speed_limit_pub = self.create_publisher(
            SpeedLimit, "/speed_limit", 10
        )
        self.create_subscription(
            LocalizationState,
            "/chassis/localization_state",
            self._on_localization,
            latched,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            ModuleState,
            "/arm/state",
            self._on_module_state,
            latched,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            ModuleState,
            "/cargo/state",
            self._on_module_state,
            latched,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            StationTargetState,
            "/perception/station_target_state",
            self._on_target,
            target_qos,
            callback_group=self._callback_group,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            10,
            callback_group=self._callback_group,
        )
        self.create_service(
            SetLockerDoor,
            "/locker/set_door_state",
            self._set_door_state,
            callback_group=self._callback_group,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._goal_lock = threading.Lock()
        self._goal_reserved = False
        self._localized = False
        self._arm_state = None
        self._cargo_state = None
        self._target = None
        self._door_states = {}
        self._minimum_scan_range = math.inf
        self._busy = False
        self._operation_state = ModuleState.STATE_UNCONFIGURED
        self._detail = "waiting for localization"
        self._prepared_routes = {}
        self._nav_distance_remaining = 0.0
        self.create_timer(0.5, self._publish_module_state)

    def _goal_callback(self, goal) -> int:
        valid_ops = {
            ExecuteChassisTask.Goal.APPROACH_PICKUP,
            ExecuteChassisTask.Goal.APPROACH_DELIVERY,
            ExecuteChassisTask.Goal.RETURN_HOME,
            ExecuteChassisTask.Goal.PREPARE_ROUTE,
        }
        if goal.operation not in valid_ops or goal.staging_pose.header.frame_id == "":
            return GoalResponse.REJECT
        with self._goal_lock:
            if self._goal_reserved:
                return GoalResponse.REJECT
            self._goal_reserved = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> int:
        return CancelResponse.ACCEPT

    def _on_localization(self, msg: LocalizationState) -> None:
        self._localized = bool(msg.localized)
        if not self._busy:
            self._operation_state = (
                ModuleState.STATE_READY
                if self._localized
                else ModuleState.STATE_UNCONFIGURED
            )
            self._detail = "ready" if self._localized else "waiting for localization"

    def _on_module_state(self, msg: ModuleState) -> None:
        if msg.module == ModuleState.MODULE_ARM:
            self._arm_state = msg
        elif msg.module == ModuleState.MODULE_CARGO:
            self._cargo_state = msg

    def _on_target(self, msg: StationTargetState) -> None:
        self._target = msg
        if msg.station_type == StationTargetState.STATION_LOCKER:
            key = (msg.station_id, int(msg.locker_slot))
            if key in self._door_states:
                msg.door_open = self._door_states[key]
        self._target_pub.publish(msg)

    def _on_scan(self, msg: LaserScan) -> None:
        finite = [r for r in msg.ranges if math.isfinite(r) and r >= msg.range_min]
        self._minimum_scan_range = min(finite) if finite else math.inf

    def _set_door_state(self, request, response):
        key = (request.locker_id, int(request.locker_slot))
        self._door_states[key] = bool(request.open)
        if (
            self._target is not None
            and self._target.station_id == request.locker_id
            and int(self._target.locker_slot) == int(request.locker_slot)
        ):
            self._target.door_open = bool(request.open)
            self._target.header.stamp = self.get_clock().now().to_msg()
            self._target_pub.publish(self._target)
        response.success = True
        response.message = "door state updated"
        return response

    def _motion_safe(self) -> bool:
        if not bool(self.get_parameter("require_module_safety_states").value):
            return True
        return bool(
            self._arm_state
            and self._cargo_state
            and self._arm_state.ready
            and self._cargo_state.ready
            and self._arm_state.transport_safe
            and self._cargo_state.transport_safe
        )

    def _publish_module_state(self) -> None:
        msg = ModuleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.module = ModuleState.MODULE_CHASSIS
        msg.lifecycle_state = (
            ModuleState.STATE_BUSY
            if self._busy
            else ModuleState.STATE_READY
            if self._localized
            else ModuleState.STATE_UNCONFIGURED
        )
        msg.operation_state = self._operation_state
        msg.ready = bool(self._localized)
        msg.busy = self._busy
        msg.transport_safe = not self._busy
        msg.detail = self._detail
        self._module_pub.publish(msg)

    def _feedback(self, goal_handle, phase: int, detail: str, progress: float) -> None:
        feedback = ExecuteChassisTask.Feedback()
        feedback.phase = phase
        feedback.detail = detail
        feedback.progress = float(max(0.0, min(1.0, progress)))
        feedback.remaining_distance_m = float(self._nav_distance_remaining)
        goal_handle.publish_feedback(feedback)

    def _result(self, success: bool, code: int, message: str):
        result = ExecuteChassisTask.Result()
        result.success = success
        result.error_code = code
        result.message = message
        result.holding_brake = success
        pose = self._base_pose()
        if pose is not None:
            result.final_base_pose = pose
        if self._target is not None and self._target.pose_valid:
            result.detected_target_pose = self._target.target_pose
            result.measured_gap_m = float(self._measured_gap(pose, self._target)) if pose else 0.0
        return result

    @staticmethod
    def _duration_seconds(duration) -> float:
        return float(duration.sec) + float(duration.nanosec) * 1.0e-9

    async def _wait_future(self, future, timeout_s: float, goal_handle=None):
        started = time.monotonic()
        while not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return None, "cancelled"
            if time.monotonic() - started > timeout_s:
                return None, "timeout"
            time.sleep(0.05)
        try:
            return future.result(), "ok"
        except Exception as exc:  # pragma: no cover - middleware errors vary
            self.get_logger().error(f"ROS future failed: {exc}")
            return None, "error"

    async def _compute_route(self, goal_handle, pose: PoseStamped, timeout_s: float):
        if not self._compute_path.wait_for_server(
            timeout_sec=float(self.get_parameter("nav_server_timeout_s").value)
        ):
            return None, self.ERR_NAV_SERVER, "ComputePathToPose unavailable"
        request = ComputePathToPose.Goal()
        request.goal = pose
        request.planner_id = str(self.get_parameter("planner_id").value)
        request.use_start = False
        goal_future = self._compute_path.send_goal_async(request)
        path_goal, status = await self._wait_future(
            goal_future, min(timeout_s, 10.0), goal_handle
        )
        if status != "ok" or path_goal is None or not path_goal.accepted:
            code = self.ERR_CANCELLED if status == "cancelled" else self.ERR_PLAN_FAILED
            return None, code, f"path goal {status}"
        result_future = path_goal.get_result_async()
        wrapped, status = await self._wait_future(
            result_future, min(timeout_s, 30.0), goal_handle
        )
        if (
            status != "ok"
            or wrapped is None
            or wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or not wrapped.result.path.poses
        ):
            code = self.ERR_CANCELLED if status == "cancelled" else self.ERR_PLAN_FAILED
            return None, code, f"path computation {status}"
        return wrapped.result.path, 0, "path ready"

    def _nav_feedback(self, msg) -> None:
        self._nav_distance_remaining = float(msg.feedback.distance_remaining)

    async def _navigate_to(self, goal_handle, pose: PoseStamped, timeout_s: float):
        if not self._navigate.wait_for_server(
            timeout_sec=float(self.get_parameter("nav_server_timeout_s").value)
        ):
            return False, self.ERR_NAV_SERVER, "NavigateToPose unavailable"
        request = NavigateToPose.Goal()
        request.pose = pose
        goal_future = self._navigate.send_goal_async(
            request, feedback_callback=self._nav_feedback
        )
        nav_goal, status = await self._wait_future(
            goal_future, min(timeout_s, 10.0), goal_handle
        )
        if status != "ok" or nav_goal is None or not nav_goal.accepted:
            code = self.ERR_CANCELLED if status == "cancelled" else self.ERR_NAV_FAILED
            return False, code, f"navigation goal {status}"

        result_future = nav_goal.get_result_async()
        started = time.monotonic()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal.cancel_goal_async()
                return False, self.ERR_CANCELLED, "navigation cancelled"
            if not self._motion_safe():
                nav_goal.cancel_goal_async()
                return False, self.ERR_MODULE_UNSAFE, "arm or cargo left transport-safe state"
            if time.monotonic() - started > timeout_s:
                nav_goal.cancel_goal_async()
                return False, self.ERR_NAV_TIMEOUT, "navigation timeout"
            self._feedback(
                goal_handle,
                ExecuteChassisTask.Feedback.APPROACH,
                "following Nav2 path",
                0.25,
            )
            time.sleep(0.1)
        wrapped = result_future.result()
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            return False, self.ERR_NAV_FAILED, f"Nav2 status {wrapped.status}"
        self._nav_distance_remaining = 0.0
        return True, 0, "navigation succeeded"

    def _simulate_target_hint(self, goal) -> None:
        if (
            not bool(self.get_parameter("use_target_hint_in_simulation").value)
            or goal.target_hint_pose.header.frame_id == ""
        ):
            return
        target = StationTargetState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = goal.target_hint_pose.header.frame_id
        target.station_type = goal.station_type
        target.station_id = goal.station_id
        target.locker_slot = goal.locker_slot
        target.station_detected = True
        target.target_detected = True
        target.pose_valid = True
        target.target_pose = goal.target_hint_pose
        target.confidence = 1.0
        target.door_open = self._door_states.get(
            (goal.station_id, int(goal.locker_slot)), False
        )
        self._target = target
        self._target_pub.publish(target)

    @staticmethod
    def _target_matches(goal, target) -> bool:
        if target is None or not target.pose_valid or not target.target_detected:
            return False
        if target.station_id != goal.station_id or target.station_type != goal.station_type:
            return False
        return not (
            goal.station_type == ExecuteChassisTask.Goal.STATION_LOCKER
            and int(target.locker_slot) != int(goal.locker_slot)
        )

    def _stop_align(self) -> None:
        self._align_pub.publish(Twist())

    async def _search(self, goal_handle, goal):
        self._simulate_target_hint(goal)
        started = time.monotonic()
        timeout = float(self.get_parameter("search_timeout_s").value)
        rate = abs(float(self.get_parameter("search_yaw_rate_rps").value))
        sweep = max(float(self.get_parameter("search_sweep_period_s").value), 0.2)
        while time.monotonic() - started <= timeout:
            if goal_handle.is_cancel_requested:
                self._stop_align()
                return False
            if self._target_matches(goal, self._target):
                self._stop_align()
                return True
            direction = 1.0 if int((time.monotonic() - started) / sweep) % 2 == 0 else -1.0
            command = Twist()
            command.angular.z = direction * rate
            self._align_pub.publish(command)
            self._feedback(
                goal_handle,
                ExecuteChassisTask.Feedback.SEARCH,
                "searching for configured station and slot",
                0.55,
            )
            time.sleep(0.05)
        self._stop_align()
        return False

    def _base_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter("map_frame").value),
                str(self.get_parameter("base_frame").value),
                rclpy.time.Time(),
            )
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def _measured_errors(self, base_pose, target):
        station_yaw = yaw_from_quaternion(target.target_pose.pose.orientation)
        base_yaw = yaw_from_quaternion(base_pose.pose.orientation)
        side = str(self.get_parameter("docking_side").value)
        side_sign = 1.0 if side == "left" else -1.0
        offset = float(self.get_parameter("base_side_offset_m").value)
        reference_x = float(self.get_parameter("side_reference_x_m").value)
        bx = (math.cos(base_yaw), math.sin(base_yaw))
        by = (-math.sin(base_yaw), math.cos(base_yaw))
        reference = (
            base_pose.pose.position.x + reference_x * bx[0] + side_sign * offset * by[0],
            base_pose.pose.position.y + reference_x * bx[1] + side_sign * offset * by[1],
        )
        delta = (
            reference[0] - target.target_pose.pose.position.x,
            reference[1] - target.target_pose.pose.position.y,
        )
        normal = (math.cos(station_yaw), math.sin(station_yaw))
        tangent = (-normal[1], normal[0])
        gap = delta[0] * normal[0] + delta[1] * normal[1]
        lateral = delta[0] * tangent[0] + delta[1] * tangent[1]
        desired_yaw = normalize_angle(
            station_yaw + side_sign * math.pi * 0.5
        )
        yaw_error = normalize_angle(desired_yaw - base_yaw)
        return gap, lateral, yaw_error

    def _measured_gap(self, base_pose, target) -> float:
        if base_pose is None:
            return 0.0
        return self._measured_errors(base_pose, target)[0]

    async def _align(self, goal_handle, goal):
        gap = float(goal.target_gap_m)
        if gap <= 0.0:
            gap = float(self.get_parameter("target_gap_m").value)
        min_gap = float(self.get_parameter("min_gap_m").value)
        max_gap = float(self.get_parameter("max_gap_m").value)
        if gap < min_gap or gap > max_gap:
            return False, self.ERR_INVALID_GOAL, "target gap outside configured range"

        side = str(self.get_parameter("docking_side").value)
        side_offset = float(self.get_parameter("base_side_offset_m").value)
        reference_x = float(self.get_parameter("side_reference_x_m").value)
        goal_pose = compute_side_goal(
            self._target.target_pose, side, side_offset, gap, reference_x
        )
        timeout = float(self.get_parameter("align_timeout_s").value)
        rate = max(float(self.get_parameter("control_rate_hz").value), 1.0)
        max_v = float(self.get_parameter("max_align_linear_mps").value)
        max_w = float(self.get_parameter("max_align_angular_rps").value)
        xy_tol = float(self.get_parameter("xy_tolerance_m").value)
        yaw_tol = float(self.get_parameter("yaw_tolerance_rad").value)
        lateral_tol = float(self.get_parameter("lateral_tolerance_m").value)
        stable_required = int(self.get_parameter("stable_cycles").value)
        hard_stop = float(self.get_parameter("hard_stop_range_m").value)
        stable = 0
        started = time.monotonic()

        while time.monotonic() - started <= timeout:
            if goal_handle.is_cancel_requested:
                self._stop_align()
                return False, self.ERR_CANCELLED, "alignment cancelled"
            if not self._motion_safe():
                self._stop_align()
                return False, self.ERR_MODULE_UNSAFE, "module left transport-safe state"
            if self._minimum_scan_range < hard_stop:
                self._stop_align()
                return False, self.ERR_OBSTACLE, "hard-stop obstacle range reached"
            base = self._base_pose()
            if base is None:
                self._stop_align()
                return False, self.ERR_TF, "map to base transform unavailable"

            dx = goal_pose.pose.position.x - base.pose.position.x
            dy = goal_pose.pose.position.y - base.pose.position.y
            distance = math.hypot(dx, dy)
            current_yaw = yaw_from_quaternion(base.pose.orientation)
            target_yaw = yaw_from_quaternion(goal_pose.pose.orientation)
            bearing_error = normalize_angle(math.atan2(dy, dx) - current_yaw)
            final_yaw_error = normalize_angle(target_yaw - current_yaw)
            measured_gap, lateral, side_yaw_error = self._measured_errors(
                base, self._target
            )
            within = (
                distance <= xy_tol
                and abs(side_yaw_error) <= yaw_tol
                and min_gap <= measured_gap <= max_gap
                and abs(lateral) <= lateral_tol
            )
            stable = stable + 1 if within else 0
            if stable >= stable_required:
                self._stop_align()
                return True, 0, "side alignment stable"

            command = Twist()
            rotate_threshold = float(
                self.get_parameter("rotate_in_place_threshold_rad").value
            )
            if distance > xy_tol:
                if abs(bearing_error) > rotate_threshold:
                    command.angular.z = max(
                        -max_w,
                        min(
                            max_w,
                            float(self.get_parameter("heading_gain").value)
                            * bearing_error,
                        ),
                    )
                else:
                    command.linear.x = max(
                        -max_v,
                        min(
                            max_v,
                            float(self.get_parameter("linear_gain").value)
                            * distance,
                        ),
                    )
                    command.angular.z = max(
                        -max_w,
                        min(
                            max_w,
                            float(self.get_parameter("heading_gain").value)
                            * bearing_error,
                        ),
                    )
            else:
                command.angular.z = max(
                    -max_w,
                    min(
                        max_w,
                        float(self.get_parameter("yaw_gain").value)
                        * final_yaw_error,
                    ),
                )
            self._align_pub.publish(command)

            feedback = ExecuteChassisTask.Feedback()
            feedback.phase = ExecuteChassisTask.Feedback.ALIGN
            feedback.detail = f"side={side}, gap={measured_gap:.3f} m"
            feedback.progress = 0.80
            feedback.remaining_distance_m = float(distance)
            feedback.lateral_error_m = float(lateral)
            feedback.yaw_error_rad = float(side_yaw_error)
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0 / rate)

        self._stop_align()
        return False, self.ERR_ALIGN_TIMEOUT, "side alignment timeout"

    def _set_speed_limit(self, speed_mps: float) -> None:
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = float(max(0.0, speed_mps))
        self._speed_limit_pub.publish(msg)

    async def _execute(self, goal_handle):
        goal = goal_handle.request
        self._busy = True
        self._operation_state = ModuleState.STATE_BUSY
        self._detail = "executing chassis task"
        try:
            if bool(self.get_parameter("localization_required").value) and not self._localized:
                goal_handle.abort()
                return self._result(False, self.ERR_NOT_LOCALIZED, "localization not ready")
            moving = goal.operation != ExecuteChassisTask.Goal.PREPARE_ROUTE
            if moving and not self._motion_safe():
                goal_handle.abort()
                return self._result(False, self.ERR_MODULE_UNSAFE, "arm/cargo not transport-safe")

            timeout = self._duration_seconds(goal.timeout)
            if timeout <= 0.0:
                timeout = float(self.get_parameter("default_action_timeout_s").value)
            self._feedback(
                goal_handle,
                ExecuteChassisTask.Feedback.PLANNING,
                "validating and computing path",
                0.05,
            )
            path, code, message = await self._compute_route(
                goal_handle, goal.staging_pose, timeout
            )
            if path is None:
                if code == self.ERR_CANCELLED:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(False, code, message)

            route_id = str(uuid.uuid4())
            self._prepared_routes[route_id] = path
            if goal.operation == ExecuteChassisTask.Goal.PREPARE_ROUTE:
                result = self._result(True, 0, "route prepared; motion was not started")
                result.prepared_route_id = route_id
                goal_handle.succeed()
                return result

            speed = float(goal.max_linear_speed_mps)
            if speed <= 0.0:
                speed = 0.15 if goal.operation == ExecuteChassisTask.Goal.RETURN_HOME else 0.30
            self._set_speed_limit(speed)
            phase = (
                ExecuteChassisTask.Feedback.BACK
                if goal.operation == ExecuteChassisTask.Goal.RETURN_HOME
                else ExecuteChassisTask.Feedback.APPROACH
            )
            self._feedback(goal_handle, phase, "starting Nav2 approach", 0.10)
            ok, code, message = await self._navigate_to(
                goal_handle, goal.staging_pose, timeout
            )
            if not ok:
                if code == self.ERR_CANCELLED:
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                return self._result(False, code, message)

            needs_align = bool(goal.requires_precision_align) and (
                goal.operation == ExecuteChassisTask.Goal.APPROACH_PICKUP
            )
            # Delivery poses are staging-only in the current frozen baseline.
            if needs_align:
                if not await self._search(goal_handle, goal):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return self._result(False, self.ERR_CANCELLED, "search cancelled")
                    goal_handle.abort()
                    return self._result(False, self.ERR_TARGET_NOT_FOUND, "station/slot not found")
                ok, code, message = await self._align(goal_handle, goal)
                if not ok:
                    if code == self.ERR_CANCELLED:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    return self._result(False, code, message)

            self._stop_align()
            self._feedback(
                goal_handle,
                ExecuteChassisTask.Feedback.HOLD,
                "stopped and holding brake",
                1.0,
            )
            goal_handle.succeed()
            result = self._result(True, 0, "chassis task succeeded")
            result.prepared_route_id = route_id
            return result
        except Exception as exc:  # pragma: no cover - defensive action boundary
            self.get_logger().error(f"chassis task crashed: {exc}")
            self._stop_align()
            goal_handle.abort()
            return self._result(False, self.ERR_INVALID_GOAL, str(exc))
        finally:
            self._stop_align()
            self._busy = False
            self._operation_state = ModuleState.STATE_READY
            self._detail = "ready"
            with self._goal_lock:
                self._goal_reserved = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ChassisTaskServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

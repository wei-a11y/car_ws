"""Top-level mission queue and the only cross-module action orchestrator."""

import queue
import threading
import time
from copy import deepcopy
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from delivery_robot_interfaces.action import (
    EjectTray,
    ExecuteChassisTask,
    PickTray,
    RetrieveTray,
    StoreTray,
)
from delivery_robot_interfaces.msg import (
    DeliveryTask,
    ModuleState,
    StationTargetState,
    TaskAcceptance,
    TaskState,
)
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .mission_planner import MissionPlanner


class TaskManager(Node):
    """Validate, plan, and execute one batch at a time."""

    def __init__(self) -> None:
        super().__init__("delivery_robot_task_manager")
        default_sites = str(
            Path(get_package_share_directory("delivery_robot_task_manager"))
            / "config"
            / "mission_sites.yaml"
        )
        for name, default in {
            "sites_file": default_sites,
            "queue_capacity": 20,
            "module_ready_timeout_s": 60.0,
            "door_open_timeout_s": 30.0,
            "action_timeout_s": 180.0,
            "pickup_target_gap_m": 0.075,
            "require_manual_door_open": True,
            "intake_frame": "intake_frame",
            "arm_enter_distance_m": 0.25,
            "arm_lift_distance_m": 0.003,
        }.items():
            self.declare_parameter(name, default)

        sites_path = Path(str(self.get_parameter("sites_file").value))
        with sites_path.open("r", encoding="utf-8") as stream:
            raw = yaml.safe_load(stream) or {}
        self._planner = MissionPlanner(raw.get("mission_sites", raw))

        latched = QoSProfile(depth=10)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        requests_qos = QoSProfile(depth=10)
        requests_qos.reliability = ReliabilityPolicy.RELIABLE
        target_qos = QoSProfile(depth=5)
        target_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self._acceptance_pub = self.create_publisher(
            TaskAcceptance, "/task_manager/task_acceptance", latched
        )
        self._state_pub = self.create_publisher(
            TaskState, "/task_manager/task_state", latched
        )
        self.create_subscription(
            DeliveryTask,
            "/task_manager/task_requests",
            self._on_task,
            requests_qos,
        )
        self.create_subscription(
            ModuleState, "/chassis/state", self._on_module_state, latched
        )
        self.create_subscription(
            ModuleState, "/arm/state", self._on_module_state, latched
        )
        self.create_subscription(
            ModuleState, "/cargo/state", self._on_module_state, latched
        )
        self.create_subscription(
            StationTargetState,
            "/chassis/station_target_state",
            self._on_target,
            target_qos,
        )

        group = ReentrantCallbackGroup()
        self._chassis = ActionClient(
            self, ExecuteChassisTask, "/chassis/execute_task", callback_group=group
        )
        self._arm = ActionClient(
            self, PickTray, "/arm/pick_tray", callback_group=group
        )
        self._store = ActionClient(
            self, StoreTray, "/cargo/store_tray", callback_group=group
        )
        self._retrieve = ActionClient(
            self, RetrieveTray, "/cargo/retrieve_tray", callback_group=group
        )
        self._eject = ActionClient(
            self, EjectTray, "/cargo/eject_tray", callback_group=group
        )

        self._queue = queue.Queue(maxsize=int(self.get_parameter("queue_capacity").value))
        self._known_tasks = set()
        self._module_states = {}
        self._station_target = None
        self._shutdown = threading.Event()
        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()
        self._publish_task_state(TaskState.INITIALIZING, detail="waiting for lower modules")

    def destroy_node(self):
        self._shutdown.set()
        return super().destroy_node()

    def _on_module_state(self, msg: ModuleState) -> None:
        self._module_states[int(msg.module)] = msg

    def _on_target(self, msg: StationTargetState) -> None:
        self._station_target = msg

    def _publish_acceptance(self, task, accepted: bool, code: int, message: str) -> None:
        reply = TaskAcceptance()
        reply.header.stamp = self.get_clock().now().to_msg()
        reply.task_id = task.task_id
        reply.revision = task.revision
        reply.accepted = accepted
        reply.reason_code = code
        reply.message = message
        reply.queue_position = self._queue.qsize() if accepted else 0
        self._acceptance_pub.publish(reply)

    def _on_task(self, msg: DeliveryTask) -> None:
        key = (msg.task_id, int(msg.revision))
        if not msg.task_id or key in self._known_tasks:
            self._publish_acceptance(
                msg, False, TaskAcceptance.DUPLICATE_TASK, "empty or duplicate task key"
            )
            return
        valid, reason = self._planner.validate_orders(msg.orders)
        if not valid:
            code = (
                TaskAcceptance.UNCALIBRATED_SITE
                if "not calibrated" in reason
                else TaskAcceptance.INVALID_TASK
            )
            self._publish_acceptance(msg, False, code, reason)
            return
        try:
            self._queue.put_nowait(deepcopy(msg))
        except queue.Full:
            self._publish_acceptance(
                msg, False, TaskAcceptance.QUEUE_FULL, "task queue is full"
            )
            return
        self._known_tasks.add(key)
        self._publish_acceptance(msg, True, TaskAcceptance.ACCEPTED, "queued")

    def _publish_task_state(
        self,
        phase: int,
        task_id: str = "",
        order_id: str = "",
        progress: float = 0.0,
        error_code: int = 0,
        detail: str = "",
    ) -> None:
        state = TaskState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.task_id = task_id
        state.phase = phase
        state.active_order_id = order_id
        state.progress = float(progress)
        state.error_code = int(error_code)
        state.detail = detail
        self._state_pub.publish(state)

    def _all_modules_safe(self) -> bool:
        for module in (
            ModuleState.MODULE_CHASSIS,
            ModuleState.MODULE_ARM,
            ModuleState.MODULE_CARGO,
        ):
            state = self._module_states.get(module)
            if state is None or not state.ready or not state.transport_safe:
                return False
        return True

    def _wait_modules_safe(self, timeout_s: float) -> bool:
        started = time.monotonic()
        while not self._shutdown.is_set() and time.monotonic() - started <= timeout_s:
            if self._all_modules_safe():
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def _wait_future(future, timeout_s: float):
        started = time.monotonic()
        while not future.done() and time.monotonic() - started <= timeout_s:
            time.sleep(0.02)
        if not future.done():
            return None
        return future.result()

    def _start_action(self, client, goal, timeout_s: float):
        if not client.wait_for_server(timeout_sec=min(timeout_s, 5.0)):
            return None, "action server unavailable"
        handle = self._wait_future(client.send_goal_async(goal), min(timeout_s, 10.0))
        if handle is None or not handle.accepted:
            return None, "action goal rejected or timed out"
        return handle, "accepted"

    def _wait_action_result(self, handle, timeout_s: float):
        wrapped = self._wait_future(handle.get_result_async(), timeout_s)
        if wrapped is None:
            handle.cancel_goal_async()
            return None, "action result timeout"
        result = wrapped.result
        if not getattr(result, "success", False):
            return result, getattr(result, "message", "action failed")
        return result, "succeeded"

    def _call_action(self, client, goal, timeout_s: float):
        handle, message = self._start_action(client, goal, timeout_s)
        if handle is None:
            return None, message
        return self._wait_action_result(handle, timeout_s)

    def _chassis_goal(self, task_id, planned, operation, precision=True):
        goal = ExecuteChassisTask.Goal()
        goal.task_id = task_id
        goal.order_id = planned.order.order_id if planned else ""
        goal.operation = operation
        goal.station_id = planned.order.locker_id if planned else "home"
        goal.station_type = (
            ExecuteChassisTask.Goal.STATION_LOCKER
            if operation in (
                ExecuteChassisTask.Goal.APPROACH_PICKUP,
                ExecuteChassisTask.Goal.PREPARE_ROUTE,
            )
            and planned
            else ExecuteChassisTask.Goal.STATION_RECEIVER
            if operation == ExecuteChassisTask.Goal.APPROACH_DELIVERY
            else ExecuteChassisTask.Goal.STATION_HOME
        )
        if planned:
            goal.staging_pose = (
                planned.locker_staging_pose
                if operation != ExecuteChassisTask.Goal.APPROACH_DELIVERY
                else planned.order.delivery_pose
            )
            goal.target_hint_pose = planned.slot_target_pose
            goal.locker_slot = int(planned.order.locker_slot)
        goal.requires_precision_align = bool(
            precision and operation == ExecuteChassisTask.Goal.APPROACH_PICKUP
        )
        goal.target_gap_m = float(self.get_parameter("pickup_target_gap_m").value)
        goal.max_linear_speed_mps = 0.30
        goal.timeout.sec = int(self.get_parameter("action_timeout_s").value)
        return goal

    def _wait_door(self, planned, timeout_s: float) -> bool:
        if not bool(self.get_parameter("require_manual_door_open").value):
            return True
        started = time.monotonic()
        while not self._shutdown.is_set() and time.monotonic() - started <= timeout_s:
            target = self._station_target
            if (
                target
                and target.station_id == planned.order.locker_id
                and int(target.locker_slot) == int(planned.order.locker_slot)
                and target.door_open
            ):
                return True
            time.sleep(0.05)
        return False

    def _next_route_goal(self, task_id, plan, pickup_index):
        if pickup_index + 1 < len(plan.pickup_sequence):
            return self._chassis_goal(
                task_id,
                plan.pickup_sequence[pickup_index + 1],
                ExecuteChassisTask.Goal.PREPARE_ROUTE,
                precision=False,
            )
        if plan.delivery_sequence:
            planned = plan.delivery_sequence[0]
            goal = ExecuteChassisTask.Goal()
            goal.task_id = task_id
            goal.order_id = planned.order.order_id
            goal.operation = ExecuteChassisTask.Goal.PREPARE_ROUTE
            goal.station_id = planned.order.order_id
            goal.station_type = ExecuteChassisTask.Goal.STATION_RECEIVER
            goal.staging_pose = planned.order.delivery_pose
            goal.timeout.sec = int(self.get_parameter("action_timeout_s").value)
            return goal
        return None

    def _fail(self, task_id: str, order_id: str, result, message: str) -> bool:
        code = int(getattr(result, "error_code", 1000)) if result is not None else 1000
        self._publish_task_state(
            TaskState.FAULT,
            task_id,
            order_id,
            error_code=code,
            detail=message,
        )
        return False

    def _execute_task(self, task: DeliveryTask) -> bool:
        timeout = float(self.get_parameter("action_timeout_s").value)
        ready_timeout = float(self.get_parameter("module_ready_timeout_s").value)
        if not self._wait_modules_safe(ready_timeout):
            return self._fail(task.task_id, "", None, "lower modules not ready and safe")

        self._publish_task_state(TaskState.PLANNING_TASK, task.task_id, detail="planning batch")
        plan = self._planner.create_plan(task.orders, bool(task.on_time_mode))
        total_steps = max(1, len(plan.pickup_sequence) * 3 + len(plan.delivery_sequence) * 3 + 1)
        completed_steps = 0

        for index, planned in enumerate(plan.pickup_sequence):
            order_id = planned.order.order_id
            self._publish_task_state(
                TaskState.PICKUP_APPROACH,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="approach, search, and side-align locker slot",
            )
            chassis_goal = self._chassis_goal(
                task.task_id, planned, ExecuteChassisTask.Goal.APPROACH_PICKUP
            )
            result, message = self._call_action(self._chassis, chassis_goal, timeout)
            if result is None or not result.success:
                return self._fail(task.task_id, order_id, result, message)
            completed_steps += 1

            self._publish_task_state(
                TaskState.WAIT_DOOR_OPEN,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="waiting for manual locker door input",
            )
            if not self._wait_door(
                planned, float(self.get_parameter("door_open_timeout_s").value)
            ):
                return self._fail(task.task_id, order_id, None, "locker door open timeout")

            arm_goal = PickTray.Goal()
            arm_goal.task_id = task.task_id
            arm_goal.order_id = order_id
            arm_goal.locker_id = planned.order.locker_id
            arm_goal.locker_slot = int(planned.order.locker_slot)
            arm_goal.slot_hint_pose = planned.slot_target_pose
            arm_goal.intake_frame = str(self.get_parameter("intake_frame").value)
            arm_goal.enter_distance_m = float(
                self.get_parameter("arm_enter_distance_m").value
            )
            arm_goal.lift_distance_m = float(
                self.get_parameter("arm_lift_distance_m").value
            )
            self._publish_task_state(
                TaskState.PICK_TRAY,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="arm pick-to-intake",
            )
            result, message = self._call_action(self._arm, arm_goal, timeout)
            if result is None or not result.success or not result.tray_at_intake:
                return self._fail(task.task_id, order_id, result, message)
            completed_steps += 1

            store_goal = StoreTray.Goal()
            store_goal.task_id = task.task_id
            store_goal.order_id = order_id
            store_goal.target_slot = int(planned.cargo_slot)
            self._publish_task_state(
                TaskState.STORE_TRAY,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="cargo store; chassis route planning may run, motion is forbidden",
            )
            store_handle, message = self._start_action(self._store, store_goal, timeout)
            if store_handle is None:
                return self._fail(task.task_id, order_id, None, message)

            # Planning-only chassis action is allowed while cargo reports
            # transport_safe=false. It never publishes a velocity command.
            preplan_goal = self._next_route_goal(task.task_id, plan, index)
            if preplan_goal is not None:
                preplan_result, preplan_message = self._call_action(
                    self._chassis, preplan_goal, timeout
                )
                if preplan_result is None or not preplan_result.success:
                    self.get_logger().warning(f"next route preplan failed: {preplan_message}")
            result, message = self._wait_action_result(store_handle, timeout)
            if result is None or not result.success or not result.transport_safe:
                return self._fail(task.task_id, order_id, result, message)
            completed_steps += 1

        for planned in plan.delivery_sequence:
            order_id = planned.order.order_id
            if not self._wait_modules_safe(ready_timeout):
                return self._fail(task.task_id, order_id, None, "modules not safe before delivery motion")
            self._publish_task_state(
                TaskState.DELIVERY_APPROACH,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="navigate to delivery staging pose; precision align disabled",
            )
            chassis_goal = self._chassis_goal(
                task.task_id,
                planned,
                ExecuteChassisTask.Goal.APPROACH_DELIVERY,
                precision=False,
            )
            result, message = self._call_action(self._chassis, chassis_goal, timeout)
            if result is None or not result.success:
                return self._fail(task.task_id, order_id, result, message)
            completed_steps += 1

            retrieve_goal = RetrieveTray.Goal()
            retrieve_goal.task_id = task.task_id
            retrieve_goal.order_id = order_id
            retrieve_goal.source_slot = int(planned.cargo_slot)
            self._publish_task_state(
                TaskState.RETRIEVE_TRAY,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="cargo retrieve to transfer T",
            )
            result, message = self._call_action(self._retrieve, retrieve_goal, timeout)
            if result is None or not result.success or not result.tray_at_transfer:
                return self._fail(task.task_id, order_id, result, message)
            completed_steps += 1

            eject_goal = EjectTray.Goal()
            eject_goal.task_id = task.task_id
            eject_goal.order_id = order_id
            eject_goal.receiver_id = order_id
            eject_goal.target_gap_m = 0.0
            self._publish_task_state(
                TaskState.EJECT_TRAY,
                task.task_id,
                order_id,
                completed_steps / total_steps,
                detail="cargo eject at staging pose",
            )
            result, message = self._call_action(self._eject, eject_goal, timeout)
            if result is None or not result.success or not result.receiver_confirmed:
                return self._fail(task.task_id, order_id, result, message)
            completed_steps += 1

        if not self._wait_modules_safe(ready_timeout):
            return self._fail(task.task_id, "", None, "modules not safe before return home")
        home_goal = ExecuteChassisTask.Goal()
        home_goal.task_id = task.task_id
        home_goal.operation = ExecuteChassisTask.Goal.RETURN_HOME
        home_goal.station_id = "home"
        home_goal.station_type = ExecuteChassisTask.Goal.STATION_HOME
        home_goal.staging_pose = plan.home_pose
        home_goal.max_linear_speed_mps = 0.15
        home_goal.timeout.sec = int(timeout)
        self._publish_task_state(
            TaskState.RETURN_HOME,
            task.task_id,
            progress=completed_steps / total_steps,
            detail="returning to home pose",
        )
        result, message = self._call_action(self._chassis, home_goal, timeout)
        if result is None or not result.success:
            return self._fail(task.task_id, "", result, message)
        self._publish_task_state(
            TaskState.COMPLETED,
            task.task_id,
            progress=1.0,
            detail="all orders delivered and robot returned home",
        )
        return True

    def _work_loop(self) -> None:
        while not self._shutdown.is_set():
            try:
                task = self._queue.get(timeout=0.2)
            except queue.Empty:
                if self._all_modules_safe():
                    self._publish_task_state(TaskState.IDLE, detail="waiting for task")
                continue
            try:
                self._execute_task(task)
            except Exception as exc:  # pragma: no cover - outer safety boundary
                self.get_logger().error(f"mission execution crashed: {exc}")
                self._publish_task_state(
                    TaskState.FAULT,
                    task.task_id,
                    error_code=1000,
                    detail=str(exc),
                )
            finally:
                self._queue.task_done()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor(num_threads=6)
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

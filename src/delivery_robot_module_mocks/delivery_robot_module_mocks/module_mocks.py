"""Deterministic arm and cargo mocks with real production action types."""

import threading
import time

import rclpy
from delivery_robot_interfaces.action import EjectTray, PickTray, RetrieveTray, StoreTray
from delivery_robot_interfaces.msg import CargoSlotState, CargoState, ModuleState
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ModuleMocks(Node):
    """Provide arm/cargo behavior while preserving transport safety semantics."""

    def __init__(self) -> None:
        super().__init__("delivery_robot_module_mocks")
        for name, default in {
            "arm_duration_s": 2.0,
            "store_duration_s": 1.5,
            "retrieve_duration_s": 1.5,
            "eject_duration_s": 1.0,
            "arm_force_failure": False,
            "cargo_force_failure": False,
        }.items():
            self.declare_parameter(name, default)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._arm_pub = self.create_publisher(ModuleState, "/arm/state", latched)
        self._cargo_pub = self.create_publisher(ModuleState, "/cargo/state", latched)
        self._cargo_detail_pub = self.create_publisher(CargoState, "/cargo/cargo_state", latched)
        self._group = ReentrantCallbackGroup()
        self._arm_server = ActionServer(
            self,
            PickTray,
            "/arm/pick_tray",
            execute_callback=self._pick,
            goal_callback=self._arm_goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self._store_server = ActionServer(
            self,
            StoreTray,
            "/cargo/store_tray",
            execute_callback=self._store,
            goal_callback=self._cargo_goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self._retrieve_server = ActionServer(
            self,
            RetrieveTray,
            "/cargo/retrieve_tray",
            execute_callback=self._retrieve,
            goal_callback=self._cargo_goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self._eject_server = ActionServer(
            self,
            EjectTray,
            "/cargo/eject_tray",
            execute_callback=self._eject,
            goal_callback=self._cargo_goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )
        self._arm_lock = threading.Lock()
        self._cargo_lock = threading.Lock()
        self._arm_reserved = False
        self._cargo_reserved = False
        self._arm_busy = False
        self._cargo_busy = False
        self._arm_detail = "mock ready"
        self._cargo_detail = "mock ready"
        self._slots = {}
        self._transfer_order = ""
        self.create_timer(0.25, self._publish_states)

    def _arm_goal(self, _goal) -> int:
        with self._arm_lock:
            if self._arm_reserved:
                return GoalResponse.REJECT
            self._arm_reserved = True
        return GoalResponse.ACCEPT

    def _cargo_goal(self, _goal) -> int:
        with self._cargo_lock:
            if self._cargo_reserved:
                return GoalResponse.REJECT
            self._cargo_reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle) -> int:
        return CancelResponse.ACCEPT

    def _module_state(self, module: int, busy: bool, detail: str) -> ModuleState:
        msg = ModuleState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.module = module
        msg.lifecycle_state = ModuleState.STATE_BUSY if busy else ModuleState.STATE_READY
        msg.operation_state = msg.lifecycle_state
        msg.ready = True
        msg.busy = busy
        msg.transport_safe = not busy
        msg.detail = detail
        return msg

    def _publish_states(self) -> None:
        self._arm_pub.publish(
            self._module_state(ModuleState.MODULE_ARM, self._arm_busy, self._arm_detail)
        )
        self._cargo_pub.publish(
            self._module_state(ModuleState.MODULE_CARGO, self._cargo_busy, self._cargo_detail)
        )
        cargo = CargoState()
        cargo.header.stamp = self.get_clock().now().to_msg()
        cargo.active_order_id = self._transfer_order
        cargo.transfer_state = 1 if self._transfer_order else 0
        cargo.transport_safe = not self._cargo_busy
        for slot_id in range(1, 6):
            slot = CargoSlotState()
            slot.slot_id = slot_id
            slot.order_id = self._slots.get(slot_id, "")
            slot.occupied = slot_id in self._slots
            slot.tray_detected = slot.occupied
            slot.mechanism_state = (
                CargoSlotState.OCCUPIED if slot.occupied else CargoSlotState.EMPTY
            )
            cargo.slots.append(slot)
        self._cargo_detail_pub.publish(cargo)

    async def _delay(self, goal_handle, duration_s: float, feedback_type, phase=1):
        steps = max(1, int(duration_s * 10.0))
        for index in range(steps):
            if goal_handle.is_cancel_requested:
                return False
            feedback = feedback_type()
            feedback.phase = phase
            if hasattr(feedback, "progress"):
                feedback.progress = float(index + 1) / float(steps)
            goal_handle.publish_feedback(feedback)
            # rclpy Humble drives coroutine Action callbacks without creating a
            # standard asyncio event loop. A short blocking wait is safe here
            # because this node runs in a MultiThreadedExecutor.
            time.sleep(duration_s / float(steps))
        return True

    async def _pick(self, goal_handle):
        self._arm_busy = True
        self._arm_detail = f"mock picking {goal_handle.request.order_id}"
        try:
            completed = await self._delay(
                goal_handle,
                float(self.get_parameter("arm_duration_s").value),
                PickTray.Feedback,
                phase=1,
            )
            result = PickTray.Result()
            if not completed:
                goal_handle.canceled()
                result.message = "mock arm cancelled"
                result.error_code = 3001
                return result
            if bool(self.get_parameter("arm_force_failure").value):
                goal_handle.abort()
                result.message = "injected mock arm failure"
                result.error_code = 3002
                return result
            result.success = True
            result.message = "mock tray placed at intake"
            result.tray_at_intake = True
            result.arm_at_home = True
            result.transport_safe = True
            goal_handle.succeed()
            return result
        finally:
            self._arm_busy = False
            self._arm_detail = "mock ready"
            with self._arm_lock:
                self._arm_reserved = False

    async def _store(self, goal_handle):
        self._cargo_busy = True
        order_id = goal_handle.request.order_id
        slot_id = int(goal_handle.request.target_slot)
        self._cargo_detail = f"mock storing {order_id} in S{slot_id}"
        try:
            completed = await self._delay(
                goal_handle,
                float(self.get_parameter("store_duration_s").value),
                StoreTray.Feedback,
            )
            result = StoreTray.Result()
            if not completed:
                goal_handle.canceled()
                result.error_code = 4001
                result.message = "mock store cancelled"
                return result
            if (
                bool(self.get_parameter("cargo_force_failure").value)
                or slot_id not in range(1, 6)
                or slot_id in self._slots
            ):
                goal_handle.abort()
                result.error_code = 4002
                result.message = "invalid or occupied cargo slot"
                return result
            self._slots[slot_id] = order_id
            result.success = True
            result.actual_slot = slot_id
            result.transport_safe = True
            result.message = "mock store succeeded"
            goal_handle.succeed()
            return result
        finally:
            self._cargo_busy = False
            self._cargo_detail = "mock ready"
            with self._cargo_lock:
                self._cargo_reserved = False

    async def _retrieve(self, goal_handle):
        self._cargo_busy = True
        order_id = goal_handle.request.order_id
        slot_id = int(goal_handle.request.source_slot)
        self._cargo_detail = f"mock retrieving {order_id} from S{slot_id}"
        try:
            completed = await self._delay(
                goal_handle,
                float(self.get_parameter("retrieve_duration_s").value),
                RetrieveTray.Feedback,
            )
            result = RetrieveTray.Result()
            if not completed:
                goal_handle.canceled()
                result.error_code = 4101
                result.message = "mock retrieve cancelled"
                return result
            if self._slots.get(slot_id) != order_id or self._transfer_order:
                goal_handle.abort()
                result.error_code = 4102
                result.message = "source slot mismatch or transfer occupied"
                return result
            del self._slots[slot_id]
            self._transfer_order = order_id
            result.success = True
            result.tray_at_transfer = True
            result.transport_safe = True
            result.message = "mock retrieve succeeded"
            goal_handle.succeed()
            return result
        finally:
            self._cargo_busy = False
            self._cargo_detail = "mock ready"
            with self._cargo_lock:
                self._cargo_reserved = False

    async def _eject(self, goal_handle):
        self._cargo_busy = True
        order_id = goal_handle.request.order_id
        self._cargo_detail = f"mock ejecting {order_id}"
        try:
            completed = await self._delay(
                goal_handle,
                float(self.get_parameter("eject_duration_s").value),
                EjectTray.Feedback,
            )
            result = EjectTray.Result()
            if not completed:
                goal_handle.canceled()
                result.error_code = 4201
                result.message = "mock eject cancelled"
                return result
            if self._transfer_order != order_id:
                goal_handle.abort()
                result.error_code = 4202
                result.message = "transfer order mismatch"
                return result
            self._transfer_order = ""
            result.success = True
            result.receiver_confirmed = True
            result.transport_safe = True
            result.message = "mock eject succeeded"
            goal_handle.succeed()
            return result
        finally:
            self._cargo_busy = False
            self._cargo_detail = "mock ready"
            with self._cargo_lock:
                self._cargo_reserved = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModuleMocks()
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

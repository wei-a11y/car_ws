"""Deterministic pickup, delivery, and cargo-slot planning."""

from copy import deepcopy
from dataclasses import dataclass
from math import hypot

from geometry_msgs.msg import PoseStamped


def pose_from_list(values, frame_id="map") -> PoseStamped:
    """Build PoseStamped from [x, y, yaw]."""
    from math import cos, sin

    if len(values) != 3:
        raise ValueError("pose must contain [x, y, yaw]")
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(values[0])
    pose.pose.position.y = float(values[1])
    pose.pose.orientation.z = sin(float(values[2]) * 0.5)
    pose.pose.orientation.w = cos(float(values[2]) * 0.5)
    return pose


def distance(a: PoseStamped, b: PoseStamped) -> float:
    return hypot(
        a.pose.position.x - b.pose.position.x,
        a.pose.position.y - b.pose.position.y,
    )


@dataclass
class PlannedOrder:
    order: object
    cargo_slot: int
    locker_staging_pose: PoseStamped
    slot_target_pose: PoseStamped


@dataclass
class MissionPlan:
    pickup_sequence: list
    delivery_sequence: list
    home_pose: PoseStamped


class MissionPlanner:
    """Plan stops without controlling any module."""

    def __init__(self, site_data: dict) -> None:
        self.site_data = site_data

    @property
    def calibrated(self) -> bool:
        return bool(self.site_data.get("calibrated", False))

    def validate_orders(self, orders) -> tuple[bool, str]:
        if not self.calibrated:
            return False, "mission_sites.yaml is not calibrated"
        if not 1 <= len(orders) <= 5:
            return False, "a task must contain 1 to 5 orders"
        ids = [order.order_id for order in orders]
        if any(not order_id for order_id in ids) or len(set(ids)) != len(ids):
            return False, "order_id must be non-empty and unique"
        lockers = self.site_data.get("lockers", {})
        for order in orders:
            if order.locker_id not in lockers:
                return False, f"unknown locker_id: {order.locker_id}"
            slots = lockers[order.locker_id].get("slots", {})
            if str(int(order.locker_slot)) not in slots:
                return False, f"locker slot is not configured: {order.locker_id}/{order.locker_slot}"
            if order.delivery_pose.header.frame_id != "map":
                return False, f"delivery pose for {order.order_id} must use map frame"
        return True, "accepted"

    @staticmethod
    def _deadline_ns(order) -> int:
        value = int(order.deadline.sec) * 1_000_000_000 + int(order.deadline.nanosec)
        return value if value > 0 else 2**63 - 1

    def create_plan(self, orders, on_time_mode: bool) -> MissionPlan:
        lockers = self.site_data["lockers"]
        home = pose_from_list(self.site_data["home_pose"])

        # Visit locker groups by nearest-neighbor distance from home. Within a
        # locker, keep slot order deterministic.
        grouped = {}
        for order in orders:
            grouped.setdefault(order.locker_id, []).append(order)
        remaining_lockers = set(grouped)
        current = home
        pickup_orders = []
        while remaining_lockers:
            locker_id = min(
                remaining_lockers,
                key=lambda item: distance(
                    current, pose_from_list(lockers[item]["staging_pose"])
                ),
            )
            pickup_orders.extend(
                sorted(grouped[locker_id], key=lambda order: int(order.locker_slot))
            )
            current = pose_from_list(lockers[locker_id]["staging_pose"])
            remaining_lockers.remove(locker_id)

        if on_time_mode:
            delivery_orders = sorted(
                orders,
                key=lambda order: (
                    self._deadline_ns(order),
                    -int(order.priority),
                    order.order_id,
                ),
            )
        else:
            # Nearest-neighbor delivery route; priority resolves close ties.
            delivery_orders = []
            remaining = list(orders)
            while remaining:
                selected = min(
                    remaining,
                    key=lambda order: (
                        distance(current, order.delivery_pose)
                        - 0.05 * int(order.priority),
                        order.order_id,
                    ),
                )
                delivery_orders.append(selected)
                current = selected.delivery_pose
                remaining.remove(selected)

        # First two delivery orders occupy directly reachable lower slots.
        slot_order = [1, 2, 3, 4, 5]
        assigned = {
            order.order_id: slot_order[index]
            for index, order in enumerate(delivery_orders)
        }
        planned = {}
        for order in orders:
            locker = lockers[order.locker_id]
            planned[order.order_id] = PlannedOrder(
                order=deepcopy(order),
                cargo_slot=assigned[order.order_id],
                locker_staging_pose=pose_from_list(locker["staging_pose"]),
                slot_target_pose=pose_from_list(
                    locker["slots"][str(int(order.locker_slot))]
                ),
            )
        return MissionPlan(
            pickup_sequence=[planned[order.order_id] for order in pickup_orders],
            delivery_sequence=[planned[order.order_id] for order in delivery_orders],
            home_pose=home,
        )

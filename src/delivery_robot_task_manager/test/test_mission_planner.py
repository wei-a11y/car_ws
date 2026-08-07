from delivery_robot_interfaces.msg import Order
from geometry_msgs.msg import PoseStamped

from delivery_robot_task_manager.mission_planner import MissionPlanner


def pose(x, y, yaw=0.0):
    result = PoseStamped()
    result.header.frame_id = "map"
    result.pose.position.x = float(x)
    result.pose.position.y = float(y)
    result.pose.orientation.w = 1.0
    return result


def order(order_id, locker, slot, x, y, priority=0, deadline=0):
    result = Order()
    result.order_id = order_id
    result.locker_id = locker
    result.locker_slot = slot
    result.delivery_pose = pose(x, y)
    result.priority = priority
    result.deadline.sec = deadline
    return result


def sites(calibrated=True):
    slot_map = {str(index): [1.0, 0.1 * index, 0.0] for index in range(1, 10)}
    return {
        "calibrated": calibrated,
        "home_pose": [0.0, 0.0, 0.0],
        "lockers": {
            "near": {"staging_pose": [1.0, 0.0, 0.0], "slots": slot_map},
            "far": {"staging_pose": [5.0, 0.0, 0.0], "slots": slot_map},
        },
    }


def test_rejects_uncalibrated_sites():
    planner = MissionPlanner(sites(calibrated=False))
    valid, reason = planner.validate_orders([order("o1", "near", 1, 1, 1)])
    assert not valid
    assert "not calibrated" in reason


def test_pickups_visit_near_locker_first_and_slots_are_deterministic():
    planner = MissionPlanner(sites())
    orders = [
        order("far", "far", 1, 8, 0),
        order("near2", "near", 2, 2, 0),
        order("near1", "near", 1, 3, 0),
    ]
    plan = planner.create_plan(orders, on_time_mode=False)
    assert [item.order.order_id for item in plan.pickup_sequence] == [
        "near1",
        "near2",
        "far",
    ]


def test_first_delivery_uses_lower_slot():
    planner = MissionPlanner(sites())
    orders = [
        order("o1", "near", 1, 10, 0),
        order("o2", "near", 2, 2, 0),
        order("o3", "near", 3, 4, 0),
    ]
    plan = planner.create_plan(orders, on_time_mode=False)
    assert plan.delivery_sequence[0].order.order_id == "o2"
    assert plan.delivery_sequence[0].cargo_slot == 1
    assert plan.delivery_sequence[1].cargo_slot == 2


def test_on_time_mode_prioritizes_deadline_then_priority():
    planner = MissionPlanner(sites())
    orders = [
        order("late", "near", 1, 1, 0, priority=5, deadline=200),
        order("urgent-low", "near", 2, 2, 0, priority=1, deadline=100),
        order("urgent-high", "near", 3, 3, 0, priority=9, deadline=100),
    ]
    plan = planner.create_plan(orders, on_time_mode=True)
    assert [item.order.order_id for item in plan.delivery_sequence] == [
        "urgent-high",
        "urgent-low",
        "late",
    ]

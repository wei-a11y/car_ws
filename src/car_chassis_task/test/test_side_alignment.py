from math import pi

import pytest
from geometry_msgs.msg import PoseStamped

from car_chassis_task.side_alignment import (
    compute_side_goal,
    quaternion_from_yaw,
    yaw_from_quaternion,
)


def station_target(x=0.0, y=0.0, yaw=0.0):
    target = PoseStamped()
    target.header.frame_id = "map"
    target.pose.position.x = x
    target.pose.position.y = y
    target.pose.orientation = quaternion_from_yaw(yaw)
    return target


@pytest.mark.parametrize(
    "side, expected_yaw", [("left", pi / 2.0), ("right", -pi / 2.0)]
)
def test_selected_side_faces_station(side, expected_yaw):
    goal = compute_side_goal(station_target(), side, 0.35, 0.075)
    assert goal.pose.position.x == pytest.approx(0.425)
    assert goal.pose.position.y == pytest.approx(0.0)
    assert yaw_from_quaternion(goal.pose.orientation) == pytest.approx(expected_yaw)


def test_longitudinal_opening_offset_is_compensated():
    goal = compute_side_goal(
        station_target(), "right", 0.35, 0.075, side_reference_x_m=0.10
    )
    assert goal.pose.position.x == pytest.approx(0.425)
    assert goal.pose.position.y == pytest.approx(0.10)


def test_rotated_station_uses_outward_normal():
    goal = compute_side_goal(station_target(1.0, 2.0, pi / 2.0), "right", 0.35, 0.05)
    assert goal.pose.position.x == pytest.approx(1.0)
    assert goal.pose.position.y == pytest.approx(2.40)
    assert yaw_from_quaternion(goal.pose.orientation) == pytest.approx(0.0)


def test_invalid_side_rejected():
    with pytest.raises(ValueError):
        compute_side_goal(station_target(), "front", 0.35, 0.075)

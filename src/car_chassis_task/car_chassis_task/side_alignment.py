"""Geometry helpers for aligning a robot side with a station face."""

from math import atan2, cos, pi, sin

from geometry_msgs.msg import Pose, PoseStamped, Quaternion


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    return (angle + pi) % (2.0 * pi) - pi


def yaw_from_quaternion(q: Quaternion) -> float:
    """Return planar yaw from a quaternion."""
    return atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a planar quaternion."""
    q = Quaternion()
    q.z = sin(yaw * 0.5)
    q.w = cos(yaw * 0.5)
    return q


def compute_side_goal(
    target: PoseStamped,
    docking_side: str,
    base_side_offset_m: float,
    target_gap_m: float,
    side_reference_x_m: float = 0.0,
) -> PoseStamped:
    """Compute base pose with the selected vehicle side facing a station.

    The target +x axis is the outward normal from the station face into the
    aisle. ``side_reference_x_m`` is the longitudinal location of the working
    opening on the selected robot side, relative to base_link.
    """
    if docking_side not in ("left", "right"):
        raise ValueError("docking_side must be 'left' or 'right'")
    if base_side_offset_m <= 0.0:
        raise ValueError("base_side_offset_m must be positive")
    if target_gap_m < 0.0:
        raise ValueError("target_gap_m must be non-negative")

    station_yaw = yaw_from_quaternion(target.pose.orientation)
    side_sign = 1.0 if docking_side == "left" else -1.0
    base_yaw = normalize_angle(station_yaw + side_sign * pi * 0.5)

    normal_x = cos(station_yaw)
    normal_y = sin(station_yaw)
    base_x_x = cos(base_yaw)
    base_x_y = sin(base_yaw)
    base_y_x = -sin(base_yaw)
    base_y_y = cos(base_yaw)

    # The selected side reference must end target_gap_m from the face.
    goal_x = (
        target.pose.position.x
        + target_gap_m * normal_x
        - side_reference_x_m * base_x_x
        - side_sign * base_side_offset_m * base_y_x
    )
    goal_y = (
        target.pose.position.y
        + target_gap_m * normal_y
        - side_reference_x_m * base_x_y
        - side_sign * base_side_offset_m * base_y_y
    )

    goal = PoseStamped()
    goal.header = target.header
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.position.z = 0.0
    goal.pose.orientation = quaternion_from_yaw(base_yaw)
    return goal


def pose_from_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    """Create a planar Pose for tests and adapters."""
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation = quaternion_from_yaw(yaw)
    return pose

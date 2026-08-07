"""Single velocity output for Nav2, precision alignment, and emergency stop."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class VelocityArbiter(Node):
    """Choose one fresh velocity source and publish the safe controller input."""

    def __init__(self) -> None:
        super().__init__("velocity_arbiter")
        self.declare_parameter("nav_topic", "/cmd_vel")
        self.declare_parameter("align_topic", "/cmd_vel_align")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("emergency_stop_topic", "/emergency_stop")
        self.declare_parameter("nav_timeout_s", 0.5)
        self.declare_parameter("align_timeout_s", 0.25)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("max_linear_mps", 0.5)
        self.declare_parameter("max_angular_rps", 1.0)

        self._nav = Twist()
        self._align = Twist()
        self._nav_at = float("-inf")
        self._align_at = float("-inf")
        self._emergency_stop = False
        self._last_owner = "none"

        self.create_subscription(
            Twist, str(self.get_parameter("nav_topic").value), self._on_nav, 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("align_topic").value), self._on_align, 10
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._on_estop,
            10,
        )
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self._owner_publisher = self.create_publisher(
            String, "/chassis/cmd_vel_owner", 10
        )
        rate = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.create_timer(1.0 / rate, self._tick)

    def _on_nav(self, msg: Twist) -> None:
        self._nav = msg
        self._nav_at = time.monotonic()

    def _on_align(self, msg: Twist) -> None:
        self._align = msg
        self._align_at = time.monotonic()

    def _on_estop(self, msg: Bool) -> None:
        self._emergency_stop = bool(msg.data)

    def _clamped(self, source: Twist) -> Twist:
        result = Twist()
        max_linear = float(self.get_parameter("max_linear_mps").value)
        max_angular = float(self.get_parameter("max_angular_rps").value)
        result.linear.x = max(-max_linear, min(max_linear, source.linear.x))
        result.angular.z = max(-max_angular, min(max_angular, source.angular.z))
        return result

    def _tick(self) -> None:
        now = time.monotonic()
        nav_fresh = now - self._nav_at <= float(
            self.get_parameter("nav_timeout_s").value
        )
        align_fresh = now - self._align_at <= float(
            self.get_parameter("align_timeout_s").value
        )

        if self._emergency_stop:
            owner, command = "emergency_stop", Twist()
        elif align_fresh:
            owner, command = "align", self._clamped(self._align)
        elif nav_fresh:
            owner, command = "nav", self._clamped(self._nav)
        else:
            owner, command = "none", Twist()

        self._publisher.publish(command)
        if owner != self._last_owner:
            owner_msg = String()
            owner_msg.data = owner
            self._owner_publisher.publish(owner_msg)
            self._last_owner = owner


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

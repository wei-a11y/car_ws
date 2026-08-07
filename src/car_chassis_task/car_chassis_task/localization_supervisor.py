"""AMCL initialization and localization quality supervision."""

import math
import time

import rclpy
from delivery_robot_interfaces.msg import LocalizationState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

from .side_alignment import quaternion_from_yaw


class LocalizationSupervisor(Node):
    """Publish a configured initial pose and decide when AMCL is trustworthy."""

    def __init__(self) -> None:
        super().__init__("localization_supervisor")
        defaults = {
            "map_frame": "map",
            "publish_initial_pose": True,
            "initial_x_m": 0.0,
            "initial_y_m": 0.0,
            "initial_yaw_rad": 0.0,
            "initial_xy_variance": 0.04,
            "initial_yaw_variance": 0.007615,
            "initial_pose_delay_s": 3.0,
            "initial_pose_repeat_count": 3,
            "initial_pose_repeat_period_s": 1.0,
            "xy_variance_threshold": 0.04,
            "yaw_variance_threshold": 0.0305,
            "stable_samples_required": 3,
            "amcl_timeout_s": 3.0,
            "global_relocalization_after_s": 45.0,
            "enable_global_relocalization": False,
            "require_scan": True,
            "require_odom": True,
            "sensor_timeout_s": 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", latched
        )
        self._state_pub = self.create_publisher(
            LocalizationState, "/chassis/localization_state", latched
        )
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._global_localization = self.create_client(
            Empty, "/reinitialize_global_localization"
        )

        self._started_at = time.monotonic()
        self._last_amcl_at = float("-inf")
        self._last_scan_at = float("-inf")
        self._last_odom_at = float("-inf")
        self._last_initial_at = float("-inf")
        self._initial_count = 0
        self._stable_count = 0
        self._xy_variance = math.inf
        self._yaw_variance = math.inf
        self._localized = False
        self._global_requested = False
        self.create_timer(0.1, self._tick)

    def _on_scan(self, _msg: LaserScan) -> None:
        self._last_scan_at = time.monotonic()

    def _on_odom(self, _msg: Odometry) -> None:
        self._last_odom_at = time.monotonic()

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_amcl_at = time.monotonic()
        covariance = msg.pose.covariance
        self._xy_variance = max(float(covariance[0]), float(covariance[7]))
        self._yaw_variance = float(covariance[35])
        covariance_ok = (
            self._xy_variance
            <= float(self.get_parameter("xy_variance_threshold").value)
            and self._yaw_variance
            <= float(self.get_parameter("yaw_variance_threshold").value)
        )
        self._stable_count = self._stable_count + 1 if covariance_ok else 0

    def _sensors_ready(self, now: float) -> bool:
        timeout = float(self.get_parameter("sensor_timeout_s").value)
        scan_ok = not bool(self.get_parameter("require_scan").value) or (
            now - self._last_scan_at <= timeout
        )
        odom_ok = not bool(self.get_parameter("require_odom").value) or (
            now - self._last_odom_at <= timeout
        )
        return scan_ok and odom_ok

    def _publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.pose.pose.position.x = float(self.get_parameter("initial_x_m").value)
        msg.pose.pose.position.y = float(self.get_parameter("initial_y_m").value)
        msg.pose.pose.orientation = quaternion_from_yaw(
            float(self.get_parameter("initial_yaw_rad").value)
        )
        xy_var = float(self.get_parameter("initial_xy_variance").value)
        yaw_var = float(self.get_parameter("initial_yaw_variance").value)
        msg.pose.covariance[0] = xy_var
        msg.pose.covariance[7] = xy_var
        msg.pose.covariance[35] = yaw_var
        self._initial_pose_pub.publish(msg)

    def _publish_state(self, now: float, sensors_ready: bool) -> None:
        state = LocalizationState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = str(self.get_parameter("map_frame").value)
        state.initial_pose_sent = self._initial_count > 0
        state.xy_variance = self._xy_variance
        state.yaw_variance = self._yaw_variance
        state.stable_sample_count = self._stable_count
        amcl_fresh = now - self._last_amcl_at <= float(
            self.get_parameter("amcl_timeout_s").value
        )
        stable_required = int(self.get_parameter("stable_samples_required").value)
        self._localized = sensors_ready and amcl_fresh and (
            self._stable_count >= stable_required
        )
        state.localized = self._localized
        if not sensors_ready:
            state.state = LocalizationState.WAITING_FOR_SENSORS
            state.detail = "waiting for scan and odometry"
        elif self._localized:
            state.state = LocalizationState.LOCALIZED
            state.detail = "AMCL covariance stable"
        elif amcl_fresh:
            state.state = LocalizationState.CONVERGING
            state.detail = "waiting for AMCL covariance convergence"
        elif self._global_requested:
            state.state = LocalizationState.CONVERGING
            state.detail = "global localization requested"
        else:
            state.state = LocalizationState.INITIALIZING
            state.detail = "waiting for AMCL pose"
        self._state_pub.publish(state)

    def _tick(self) -> None:
        now = time.monotonic()
        sensors_ready = self._sensors_ready(now)
        delay = float(self.get_parameter("initial_pose_delay_s").value)
        period = float(self.get_parameter("initial_pose_repeat_period_s").value)
        repeat = int(self.get_parameter("initial_pose_repeat_count").value)
        if (
            bool(self.get_parameter("publish_initial_pose").value)
            and sensors_ready
            and now - self._started_at >= delay
            and self._initial_count < repeat
            and now - self._last_initial_at >= period
        ):
            self._publish_initial_pose()
            self._initial_count += 1
            self._last_initial_at = now

        global_after = float(
            self.get_parameter("global_relocalization_after_s").value
        )
        if (
            bool(self.get_parameter("enable_global_relocalization").value)
            and sensors_ready
            and not self._localized
            and not self._global_requested
            and now - self._started_at >= global_after
            and self._global_localization.service_is_ready()
        ):
            self._global_localization.call_async(Empty.Request())
            self._global_requested = True
        self._publish_state(now, sensors_ready)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

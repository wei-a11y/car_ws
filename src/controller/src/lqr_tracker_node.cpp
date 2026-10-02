// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/lqr_tracker.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace controller
{
class LqrTrackerNode : public rclcpp::Node
{
public:
  LqrTrackerNode()
  : Node("lqr_controller")
  {
    frame_ = parameter<std::string>("planning_frame");
    base_ = parameter<std::string>("base_frame");
    odom_frame_ = parameter<std::string>("odom_frame");
    if (frame_.empty() || base_.empty() || odom_frame_.empty() || frame_ == base_ ||
      odom_frame_ == base_ || parameter<std::string>("pose_source") != "tf" ||
      parameter<bool>("allow_odom_fallback"))
    {
      throw std::invalid_argument(
              "Verified frames and TF pose_source are required; no odom fallback");
    }
    TrackerConfig c;
    c.control_period = parameter<double>("control_period");
    c.v_nominal = parameter<double>("v_nominal");
    c.v_max = parameter<double>("v_max");
    c.w_max = parameter<double>("w_max");
    c.q_lateral = parameter<double>("q_lateral");
    c.q_heading = parameter<double>("q_heading");
    c.r_angular = parameter<double>("r_angular");
    c.riccati_tolerance = parameter<double>("riccati_tolerance");
    c.riccati_max_iterations = count("riccati_max_iterations");
    c.lookahead_distance = parameter<double>("lookahead_distance");
    c.nearest_forward_distance = parameter<double>("nearest_forward_distance");
    c.nearest_backward_distance = parameter<double>("nearest_backward_distance");
    c.slowdown_gain = parameter<double>("slowdown_gain");
    c.align_gain = parameter<double>("align_gain");
    c.heading_tolerance = parameter<double>("heading_tolerance");
    c.align_trigger = parameter<double>("align_trigger");
    c.goal_position_tolerance = parameter<double>("goal_position_tolerance");
    c.stopped_linear_tolerance = parameter<double>("stopped_linear_tolerance");
    c.stopped_angular_tolerance = parameter<double>("stopped_angular_tolerance");
    c.max_tracking_error = parameter<double>("max_tracking_error");
    c.max_path_points = count("max_path_points");
    max_path_points_ = c.max_path_points;
    tracker_ = std::make_unique<LqrTracker>(c);
    period_ = c.control_period;
    tf_timeout_ = parameter<double>("tf_timeout_sec");
    tf_max_age_ = parameter<double>("tf_max_age_sec");
    odom_max_age_ = parameter<double>("odom_max_age_sec");
    max_control_gap_ = parameter<double>("max_control_gap_sec");
    watchdog_timeout_ = parameter<double>("wall_watchdog_timeout_sec");
    for (double value : {tf_max_age_, odom_max_age_, max_control_gap_, watchdog_timeout_}) {
      if (!std::isfinite(value) || value <= period_) {
        throw std::invalid_argument(
                "Freshness/watchdog limits must be finite and exceed control_period");
      }
    }
    if (!std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 || tf_timeout_ >= period_ / 2.0) {
      throw std::invalid_argument("TF wait must be finite, nonnegative and less than half a period");
    }
    const auto stable_gain = tracker_->gain();
    if (max_control_gap_ * stable_gain[1] >= 2.0 ||
      stable_gain[1] <= 0.5 * c.v_nominal * max_control_gap_ * stable_gain[0])
    {
      throw std::invalid_argument("max_control_gap_sec exceeds the local stability range");
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, true);
    const auto qos = rclcpp::QoS(1).reliable();
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(topic("raw_cmd_topic"), qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      topic("status_topic"), rclcpp::QoS(1).reliable().transient_local());
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(topic("debug_topic"), qos);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      topic("path_topic"), qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) {on_path(*message);});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic("odom_topic"), rclcpp::SensorDataQoS().keep_last(5),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {on_odom(*message);});
    const std::string output = command_pub_->get_topic_name();
    // These are confirmed base inputs. Phase 6 must not bypass the not-yet-implemented Safety Gate.
    if (output == "/cmd_vel" || output == "/diff_drive_controller/cmd_vel_unstamped") {
      throw std::invalid_argument("LQR output must be a separate raw topic, never the base command");
    }
    const std::array<std::string, 5> topics = {output, status_pub_->get_topic_name(),
      debug_pub_->get_topic_name(), path_sub_->get_topic_name(), odom_sub_->get_topic_name()};
    for (std::size_t i = 0; i < topics.size(); ++i) {
      for (std::size_t j = i + 1; j < topics.size(); ++j) {
        if (topics[i] == topics[j]) {
          throw std::invalid_argument("Tracker topics must be distinct after remapping");
        }
      }
    }
    last_tick_ = std::chrono::steady_clock::now();
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(period_),
      [this]() {on_tick();});
    watchdog_ = create_wall_timer(
      std::chrono::duration<double>(period_), [this]() {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - last_tick_).count() >
        watchdog_timeout_)
        {
          stop("CLOCK_UNAVAILABLE", "Control clock stalled; publishing zero from wall watchdog");
        }
      });
    const auto gain = tracker_->gain();
    RCLCPP_INFO(
      get_logger(), "LQR K=[%.6f, %.6f], T=%.3f s, v_nom=%.3f m/s; %s -> %s",
      gain[0], gain[1], period_, c.v_nominal, path_sub_->get_topic_name(), output.c_str());
    stop(
      "WAIT_PATH",
      "No path; Safety Gate and verified localization are deployment prerequisites");
  }

private:
  template<typename T>
  T parameter(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, descriptor);
  }

  std::size_t count(const std::string & name)
  {
    const auto value = parameter<std::int64_t>(name);
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  std::string topic(const std::string & name)
  {
    const auto value = parameter<std::string>(name);
    if (value.empty()) {
      throw std::invalid_argument(name + " must not be empty");
    }
    return value;
  }

  static bool valid_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    constexpr double kTolerance = 1e-6;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    return std::isfinite(norm) && std::abs(norm - 1.0) <= kTolerance;
  }

  void command(Velocity2D velocity)
  {
    geometry_msgs::msg::Twist message;
    message.linear.x = velocity.v;
    message.angular.z = velocity.w;
    command_pub_->publish(message);
  }

  void status(const std::string & code, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = code;
    status_pub_->publish(message);
    if (code != last_status_) {
      RCLCPP_INFO(get_logger(), "%s: %s", code.c_str(), detail.c_str());
      last_status_ = code;
    }
  }

  void stop(const std::string & code, const std::string & detail)
  {
    command({0.0, 0.0});
    status(code, detail);
  }

  void on_path(const nav_msgs::msg::Path & message)
  {
    tracker_->clear_path();
    path_error_ = "PATH_INVALID";
    command({0.0, 0.0});  // A replacement or rejection may never retain the previous command.
    try {
      if (message.header.frame_id != frame_ || message.poses.size() > max_path_points_) {
        throw std::invalid_argument("Path requires verified planning_frame and bounded point count");
      }
      std::vector<Point2D> points;
      points.reserve(message.poses.size());
      for (const auto & pose : message.poses) {
        if (pose.header.frame_id != frame_ || !valid_quaternion(pose.pose.orientation) ||
          !std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y) ||
          !std::isfinite(pose.pose.position.z))
        {
          throw std::invalid_argument("Path poses require matching frames and finite valid poses");
        }
        points.push_back({pose.pose.position.x, pose.pose.position.y});
      }
      tracker_->set_path(points);
      path_error_ = "PATH_UNAVAILABLE";
      status(
        tracker_->has_path() ? "PATH_ACCEPTED" : "PATH_UNAVAILABLE",
        "Received " + std::to_string(points.size()) +
        " points; progress reset, no Path age timeout");
    } catch (const std::exception & error) {
      stop(path_error_, error.what());
    }
  }

  void on_odom(const nav_msgs::msg::Odometry & message)
  {
    const auto & velocity = message.twist.twist;
    if (message.header.frame_id != odom_frame_ || message.child_frame_id != base_ ||
      !std::isfinite(velocity.linear.x) || !std::isfinite(velocity.linear.y) ||
      !std::isfinite(velocity.linear.z) || !std::isfinite(velocity.angular.x) ||
      !std::isfinite(velocity.angular.y) || !std::isfinite(velocity.angular.z))
    {
      odom_.reset();
      return;
    }
    odom_ = message;
  }

  bool fresh(const builtin_interfaces::msg::Time & stamp, double limit, const rclcpp::Time & time)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
      return false;
    }
    const auto age = (time - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return std::isfinite(age) && std::abs(age) <= limit;
  }

  void on_tick()
  {
    last_tick_ = std::chrono::steady_clock::now();
    const auto time = now();
    if (time.nanoseconds() <= 0 || (last_ros_time_ &&
      ((time - *last_ros_time_).seconds() <= 0.0 ||
      (time - *last_ros_time_).seconds() > max_control_gap_)))
    {
      last_ros_time_ = time;
      stop("CLOCK_UNAVAILABLE", "Invalid control clock or excessive control gap");
      return;
    }
    last_ros_time_ = time;
    if (!tracker_->has_path()) {
      stop(path_error_, "A valid nonempty Path is required");
      return;
    }
    if (path_sub_->get_publisher_count() == 0) {
      tracker_->clear_path();
      path_error_ = "PATH_UNAVAILABLE";
      stop(path_error_, "Path publisher disappeared; a new Path is required");
      return;
    }
    if (!odom_ || !fresh(odom_->header.stamp, odom_max_age_, time)) {
      stop(
        "ODOM_UNAVAILABLE",
        "Velocity/stop confirmation requires fresh odom with verified frames");
      return;
    }
    try {
      const auto transform = buffer_->lookupTransform(
        frame_, base_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
      const auto & position = transform.transform.translation;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
        !valid_quaternion(transform.transform.rotation) ||
        ((transform.header.stamp.sec != 0 || transform.header.stamp.nanosec != 0) &&
        !fresh(transform.header.stamp, tf_max_age_, time)))
      {
        throw std::runtime_error("Robot TF is invalid, stale or in the future");
      }
      const auto result = tracker_->step(
        {position.x, position.y, tf2::getYaw(transform.transform.rotation)},
        {odom_->twist.twist.linear.x, odom_->twist.twist.angular.z});
      command(result.command);
      status(state_name(result.state), "See debug values; every fault/reached state commands zero");
      std_msgs::msg::Float64MultiArray debug;
      debug.layout.dim.resize(1);
      debug.layout.dim[0].label =
        "ey,etheta,progress,remaining,v_ref,v_cmd,w_cmd,nearest_x,nearest_y,lookahead_x,lookahead_y";
      debug.layout.dim[0].size = debug.layout.dim[0].stride = 11;
      debug.data = {result.lateral_error, result.heading_error, result.progress, result.remaining,
        result.reference_speed, result.command.v, result.command.w,
        result.nearest.x, result.nearest.y, result.lookahead.x, result.lookahead.y};
      debug_pub_->publish(debug);
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ey=%.4f m etheta=%.4f rad s=%.3f remain=%.3f v=%.3f w=%.3f preview=(%.3f,%.3f)",
        result.lateral_error, result.heading_error, result.progress, result.remaining,
        result.command.v, result.command.w, result.lookahead.x, result.lookahead.y);
    } catch (const std::exception & error) {
      stop("TF_UNAVAILABLE", error.what());
    }
  }

  std::string frame_, base_, odom_frame_, last_status_, path_error_ = "WAIT_PATH";
  double period_, tf_timeout_, tf_max_age_, odom_max_age_, max_control_gap_, watchdog_timeout_;
  std::size_t max_path_points_;
  std::unique_ptr<LqrTracker> tracker_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::optional<nav_msgs::msg::Odometry> odom_;
  std::optional<rclcpp::Time> last_ros_time_;
  std::chrono::steady_clock::time_point last_tick_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_, watchdog_;
};
}  // namespace controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<controller::LqrTrackerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("lqr_controller"), "Startup failed: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

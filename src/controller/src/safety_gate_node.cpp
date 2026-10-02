// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/safety_gate.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace controller
{
class SafetyGateNode : public rclcpp::Node
{
public:
  SafetyGateNode()
  : Node("safety_gate")
  {
    enabled_ = parameter<bool>("enable");
    base_ = parameter<std::string>("base_frame");
    scan_frame_ = parameter<std::string>("scan_frame");
    if (base_.empty() || scan_frame_.empty() || base_ == scan_frame_) {
      throw std::invalid_argument("Distinct verified base_frame and scan_frame are required");
    }
    gate_ = std::make_unique<SafetyGate>(
      SafetyConfig{
        parameter<double>("forward_sector_half_angle"), parameter<double>("stop_distance"),
        parameter<double>("release_distance"), parameter<double>("max_beam_gap"),
        parameter<bool>("allow_positive_infinity")});
    period_ = parameter<double>("publish_period");
    raw_timeout_ = parameter<double>("raw_timeout_sec");
    scan_timeout_ = parameter<double>("scan_timeout_sec");
    clock_timeout_ = parameter<double>("clock_timeout_sec");
    future_tolerance_ = parameter<double>("future_tolerance_sec");
    tf_timeout_ = parameter<double>("tf_timeout_sec");
    v_max_ = parameter<double>("v_max");
    w_max_ = parameter<double>("w_max");
    if (!std::isfinite(period_) || period_ <= 0.0 || !std::isfinite(v_max_) || v_max_ <= 0.0 ||
      !std::isfinite(w_max_) || w_max_ <= 0.0 || !std::isfinite(future_tolerance_) ||
      future_tolerance_ < 0.0 || future_tolerance_ >= std::min(raw_timeout_, scan_timeout_) ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 || tf_timeout_ >= period_ / 2.0)
    {
      throw std::invalid_argument("Invalid period, velocity, timestamp or TF limits");
    }
    for (double value : {raw_timeout_, scan_timeout_, clock_timeout_}) {
      if (!std::isfinite(value) || value <= period_) {
        throw std::invalid_argument("Input/clock timeouts must be finite and exceed publish_period");
      }
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, true);
    const auto qos = rclcpp::QoS(1).reliable();
    command_pub_ = create_publisher<geometry_msgs::msg::Twist>(topic("cmd_vel_topic"), qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      topic("status_topic"), rclcpp::QoS(1).reliable().transient_local());
    debug_pub_ = create_publisher<std_msgs::msg::Float64>(topic("debug_topic"), qos);
    raw_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      topic("raw_cmd_topic"), qos,
      [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {on_raw(*message);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      topic("scan_topic"), rclcpp::SensorDataQoS().keep_last(5),
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {on_scan(*message);});
    const std::vector<std::string> topics = {command_pub_->get_topic_name(),
      raw_sub_->get_topic_name(), scan_sub_->get_topic_name(),
      status_pub_->get_topic_name(), debug_pub_->get_topic_name()};
    for (std::size_t i = 0; i < topics.size(); ++i) {
      for (std::size_t j = i + 1; j < topics.size(); ++j) {
        if (topics[i] == topics[j]) {
          throw std::invalid_argument("Safety topics must be distinct after remapping");
        }
      }
    }
    last_clock_progress_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::duration<double>(period_), [this]() {on_tick();});
    stop(enabled_ ? "WAIT_SCAN" : "DISABLED");
    RCLCPP_INFO(
      get_logger(), "%s + %s -> %s; forward sector in %s via TF from %s; enable=%s",
      raw_sub_->get_topic_name(), scan_sub_->get_topic_name(), command_pub_->get_topic_name(),
      base_.c_str(), scan_frame_.c_str(), enabled_ ? "true" : "false (zero, not bypass)");
  }

private:
  using Steady = std::chrono::steady_clock;

  template<typename T>
  T parameter(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, descriptor);
  }

  std::string topic(const std::string & name)
  {
    const auto value = parameter<std::string>(name);
    if (value.empty()) {
      throw std::invalid_argument(name + " must not be empty");
    }
    return value;
  }

  static double age(Steady::time_point time)
  {
    return std::chrono::duration<double>(Steady::now() - time).count();
  }

  bool fresh(const rclcpp::Time & stamp, double limit)
  {
    const double elapsed = (now() - stamp).seconds();
    return now().nanoseconds() > 0 && stamp.nanoseconds() > 0 &&
           elapsed >= -future_tolerance_ && elapsed <= limit;
  }

  void publish(const geometry_msgs::msg::Twist & command, const std::string & code)
  {
    command_pub_->publish(command);
    std_msgs::msg::String message;
    message.data = code;
    status_pub_->publish(message);
    if (code != last_status_) {
      RCLCPP_INFO(get_logger(), "Safety Gate: %s", code.c_str());
      last_status_ = code;
    }
  }

  void stop(const std::string & code)
  {
    publish(geometry_msgs::msg::Twist{}, code);
  }

  void on_raw(const geometry_msgs::msg::Twist & message)
  {
    raw_.reset();
    const auto & v = message.linear;
    const auto & w = message.angular;
    if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
      !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z) ||
      v.y != 0.0 || v.z != 0.0 || w.x != 0.0 || w.y != 0.0 ||
      v.x < 0.0 || v.x > v_max_ || std::abs(w.z) > w_max_)
    {
      raw_error_ = "RAW_INVALID";
      stop(raw_error_);
      return;
    }
    raw_ = message;
    raw_time_ = now();
    raw_received_ = Steady::now();
    raw_error_ = "RAW_TIMEOUT";
    // A valid zero command should not wait for the next publish tick.
    if (v.x == 0.0 && w.z == 0.0) {
      stop("RAW_ZERO");
    }
  }

  void reject_scan(const std::string & reason)
  {
    gate_->invalidate();
    scan_time_.reset();
    scan_error_ = reason;
    std_msgs::msg::Float64 message;
    message.data = std::numeric_limits<double>::quiet_NaN();
    debug_pub_->publish(message);
    stop(reason);
  }

  void on_scan(const sensor_msgs::msg::LaserScan & message)
  {
    if (message.header.frame_id != scan_frame_ || message.header.stamp.sec < 0 ||
      message.header.stamp.nanosec >= 1000000000U)
    {
      reject_scan("SCAN_INVALID");
      return;
    }
    const rclcpp::Time stamp(message.header.stamp, get_clock()->get_clock_type());
    if (!fresh(stamp, scan_timeout_)) {
      reject_scan("SCAN_TIMEOUT");
      return;
    }
    double yaw;
    try {
      const auto transform = buffer_->lookupTransform(
        base_, scan_frame_, stamp, rclcpp::Duration::from_seconds(tf_timeout_));
      const auto & q = transform.transform.rotation;
      const auto & t = transform.transform.translation;
      const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
      constexpr double kQuaternionTolerance = 1e-6;
      constexpr double kPlanarTolerance = 1e-3;
      if (!std::isfinite(norm) || std::abs(norm - 1.0) > kQuaternionTolerance) {
        reject_scan("TF_INVALID");
        return;
      }
      double roll, pitch;
      tf2::Quaternion rotation;
      tf2::fromMsg(q, rotation);
      tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
      if (!std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z) ||
        !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw) ||
        std::abs(roll) > kPlanarTolerance || std::abs(pitch) > kPlanarTolerance)
      {
        reject_scan("TF_INVALID");
        return;
      }
    } catch (const tf2::TransformException &) {
      reject_scan("TF_UNAVAILABLE");
      return;
    }
    const auto assessment = gate_->update(
      {
        message.angle_min, message.angle_max, message.angle_increment,
        message.range_min, message.range_max, yaw, message.ranges});
    if (!assessment.valid) {
      reject_scan("SCAN_INVALID");
      return;
    }
    scan_time_ = stamp;
    scan_received_ = Steady::now();
    scan_error_ = "SCAN_TIMEOUT";
    std_msgs::msg::Float64 debug;
    debug.data = assessment.minimum_range;
    debug_pub_->publish(debug);
    if (gate_->blocked()) {
      stop("OBSTACLE_STOP");
    }
  }

  void on_tick()
  {
    const auto time = now();
    if (!enabled_) {
      gate_->invalidate();
      stop("DISABLED");
      return;
    }
    if (last_ros_time_ && (time < *last_ros_time_ ||
      (time - *last_ros_time_).seconds() > clock_timeout_))
    {
      gate_->invalidate();
      raw_.reset();
      scan_time_.reset();
      last_ros_time_ = time;
      last_clock_progress_ = Steady::now();
      stop("CLOCK_RESET");
      return;
    }
    if (!last_ros_time_ || time > *last_ros_time_) {
      last_clock_progress_ = Steady::now();
    }
    last_ros_time_ = time;
    if (time.nanoseconds() <= 0 || age(last_clock_progress_) > clock_timeout_) {
      gate_->invalidate();
      raw_.reset();
      scan_time_.reset();
      stop("CLOCK_UNAVAILABLE");
    } else if (!scan_time_ || !fresh(*scan_time_, scan_timeout_) ||
      age(scan_received_) > scan_timeout_)
    {
      gate_->invalidate();
      stop(scan_error_);
    } else if (gate_->blocked()) {
      stop("OBSTACLE_STOP");
    } else if (!raw_ || !fresh(*raw_time_, raw_timeout_) || age(raw_received_) > raw_timeout_) {
      stop(raw_error_);
    } else {
      publish(*raw_, "CLEAR");
    }
  }

  bool enabled_;
  std::string base_, scan_frame_, last_status_;
  std::string raw_error_ = "WAIT_RAW", scan_error_ = "WAIT_SCAN";
  double period_, raw_timeout_, scan_timeout_, clock_timeout_, future_tolerance_, tf_timeout_;
  double v_max_, w_max_;
  Steady::time_point raw_received_, scan_received_, last_clock_progress_;
  std::optional<rclcpp::Time> raw_time_, scan_time_, last_ros_time_;
  std::optional<geometry_msgs::msg::Twist> raw_;
  std::unique_ptr<SafetyGate> gate_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr debug_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<controller::SafetyGateNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("safety_gate"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

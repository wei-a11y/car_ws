// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/planning_grid.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plan
{
class PlanningGridNode : public rclcpp::Node
{
public:
  PlanningGridNode()
  : Node("planning_grid")
  {
    const auto input_topic = read_parameter<std::string>("map_topic");
    const auto raw_topic = read_parameter<std::string>("raw_grid_topic");
    const auto inflated_topic = read_parameter<std::string>("inflated_grid_topic");
    expected_frame_ = read_parameter<std::string>("expected_frame");
    const auto threshold = read_parameter<std::int64_t>("occupied_threshold");
    const auto max_cells = read_parameter<std::int64_t>("max_cells");
    if (threshold < 1 || threshold > 100 || max_cells <= 0) {
      throw std::invalid_argument("occupied_threshold must be [1,100]; max_cells must be positive");
    }
    config_ = {
      static_cast<int>(threshold), read_parameter<bool>("unknown_is_obstacle"),
      read_parameter<double>("robot_radius"), read_parameter<double>("safety_margin"),
      read_parameter<bool>("inflate_map_boundary"), static_cast<std::size_t>(max_cells)};
    PlanningGrid::validate_config(config_);
    if (input_topic.empty() || raw_topic.empty() || inflated_topic.empty()) {
      throw std::invalid_argument("Grid topic names must not be empty");
    }
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    raw_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(raw_topic, qos);
    inflated_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(inflated_topic, qos);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic, qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {on_map(std::move(msg));});
    const std::string resolved_input = map_sub_->get_topic_name();
    const std::string resolved_raw = raw_pub_->get_topic_name();
    const std::string resolved_inflated = inflated_pub_->get_topic_name();
    if (resolved_input == resolved_raw || resolved_input == resolved_inflated ||
      resolved_raw == resolved_inflated)
    {
      throw std::invalid_argument("Input/raw/inflated topics must be distinct after remapping");
    }
    RCLCPP_INFO(
      get_logger(), "Planning grid ready: %s -> %s / %s; robot radius %.3f + margin %.3f m",
      resolved_input.c_str(), resolved_raw.c_str(), resolved_inflated.c_str(),
      config_.robot_radius, config_.safety_margin);
  }

private:
  template<typename T>
  T read_parameter(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, descriptor);
  }

  void on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    try {
      if (msg->header.frame_id.empty() ||
        (!expected_frame_.empty() && msg->header.frame_id != expected_frame_))
      {
        throw std::invalid_argument("Map frame is empty or differs from expected_frame");
      }
      const auto & origin = msg->info.origin;
      const auto & q = origin.orientation;
      constexpr double kQuaternionTolerance = 1e-6;
      const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
      if (!std::isfinite(origin.position.z) || !std::isfinite(norm) ||
        std::abs(norm - 1.0) > kQuaternionTolerance ||
        std::abs(q.x) > kQuaternionTolerance || std::abs(q.y) > kQuaternionTolerance)
      {
        throw std::invalid_argument("Map origin requires a finite, unit, yaw-only quaternion");
      }
      if (msg->info.height == 0 || msg->info.width == 0 ||
        msg->info.width > config_.max_cells / msg->info.height)
      {
        throw std::invalid_argument("Map dimensions are zero or exceed max_cells");
      }
      if (msg->data.size() != static_cast<std::size_t>(msg->info.width) * msg->info.height) {
        throw std::invalid_argument("Occupancy data length differs from width * height");
      }
      const GridGeometry geometry = {
        msg->info.width, msg->info.height, msg->info.resolution,
        origin.position.x, origin.position.y, 2.0 * std::atan2(q.z, q.w)};
      auto grid = std::make_unique<PlanningGrid>(geometry, msg->data, config_);
      nav_msgs::msg::OccupancyGrid inflated = *msg;
      inflated.data = grid->inflated_grid();
      raw_pub_->publish(*msg);
      inflated_pub_->publish(inflated);
      latest_grid_ = std::move(grid);
      RCLCPP_DEBUG(get_logger(), "Updated %zu x %zu grid", geometry.width, geometry.height);
    } catch (const std::exception & error) {
      // Keep the last valid snapshot and its original stamp; do not fabricate fresh data.
      RCLCPP_ERROR(get_logger(), "Rejected map (last valid grid retained): %s", error.what());
    }
  }

  GridConfig config_;
  std::string expected_frame_;
  std::unique_ptr<PlanningGrid> latest_grid_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr raw_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
};
}  // namespace plan

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<plan::PlanningGridNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("planning_grid"), "Startup failed: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

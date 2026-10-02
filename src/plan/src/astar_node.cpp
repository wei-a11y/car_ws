// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/astar.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace plan
{
class AStarNode : public rclcpp::Node
{
public:
  AStarNode()
  : Node("planner")
  {
    planning_frame_ = read_parameter<std::string>("planning_frame");
    base_frame_ = read_parameter<std::string>("base_frame");
    tf_timeout_ = read_parameter<double>("tf_timeout_sec");
    tf_max_age_ = read_parameter<double>("tf_max_age_sec");
    const auto limit = read_parameter<std::int64_t>("max_cells");
    if (planning_frame_.empty() || base_frame_.empty() || limit <= 0 ||
      !std::isfinite(tf_timeout_) || tf_timeout_ < 0.0 ||
      !std::isfinite(tf_max_age_) || tf_max_age_ <= 0.0)
    {
      throw std::invalid_argument("Set verified planning_frame/base_frame and valid limits");
    }
    max_cells_ = static_cast<std::size_t>(limit);
    const auto map_topic = read_parameter<std::string>("inflated_grid_topic");
    const auto goal_topic = read_parameter<std::string>("goal_topic");
    const auto path_topic = read_parameter<std::string>("path_topic");
    const auto status_topic = read_parameter<std::string>("status_topic");
    if (map_topic.empty() || goal_topic.empty() || path_topic.empty() || status_topic.empty()) {
      throw std::invalid_argument("Planner topic names must not be empty");
    }
    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, true);
    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    const auto volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, volatile_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, latched_qos);
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, latched_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {on_map(std::move(msg));});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, volatile_qos,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {on_goal(*msg);});
    const std::string resolved_map = map_sub_->get_topic_name();
    const std::string resolved_goal = goal_sub_->get_topic_name();
    const std::string resolved_path = path_pub_->get_topic_name();
    const std::string resolved_status = status_pub_->get_topic_name();
    if (resolved_map == resolved_goal || resolved_path == resolved_status ||
      resolved_map == resolved_path || resolved_map == resolved_status ||
      resolved_goal == resolved_path || resolved_goal == resolved_status)
    {
      throw std::invalid_argument("Planner topics must be distinct after remapping");
    }
    RCLCPP_INFO(
      get_logger(), "A* ready: frame=%s, base=%s, inflated map=%s, goal=%s, path=%s",
      planning_frame_.c_str(), base_frame_.c_str(), resolved_map.c_str(),
      resolved_goal.c_str(), resolved_path.c_str());
  }

private:
  template<typename T>
  T read_parameter(const std::string & name)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    return declare_parameter<T>(name, descriptor);
  }

  static bool unit_quaternion(const geometry_msgs::msg::Quaternion & q)
  {
    constexpr double kTolerance = 1e-6;
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    return std::isfinite(norm) && std::abs(norm - 1.0) <= kTolerance;
  }

  static bool finite_pose(const geometry_msgs::msg::Pose & pose)
  {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
           std::isfinite(pose.position.z) && unit_quaternion(pose.orientation);
  }

  void clear_path()
  {
    nav_msgs::msg::Path empty;
    empty.header.frame_id = planning_frame_;
    empty.header.stamp = now();
    path_pub_->publish(empty);
    path_valid_ = false;
  }

  void publish_status(PlanStatus status, const std::string & detail)
  {
    std_msgs::msg::String message;
    message.data = status_name(status);
    status_pub_->publish(message);
    RCLCPP_INFO(get_logger(), "%s: %s", message.data.c_str(), detail.c_str());
  }

  void fail(PlanStatus status, const std::string & detail)
  {
    clear_path();
    publish_status(status, detail);
  }

  void on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
  {
    try {
      const auto & origin = msg->info.origin;
      constexpr double kTolerance = 1e-6;
      if (msg->header.frame_id != planning_frame_ || !finite_pose(origin) ||
        std::abs(origin.orientation.x) > kTolerance ||
        std::abs(origin.orientation.y) > kTolerance || msg->info.height == 0 ||
        msg->info.width == 0 || msg->info.width > max_cells_ / msg->info.height ||
        msg->data.size() != static_cast<std::size_t>(msg->info.width) * msg->info.height)
      {
        throw std::invalid_argument("Invalid inflated map frame, origin, dimensions or data size");
      }
      const GridGeometry geometry = {
        msg->info.width, msg->info.height, msg->info.resolution,
        origin.position.x, origin.position.y,
        2.0 * std::atan2(origin.orientation.z, origin.orientation.w)};
      grid_ = std::make_unique<PlanningGrid>(
        PlanningGrid::from_inflated_grid(geometry, msg->data, max_cells_));
      map_z_ = origin.position.z;
      // A new snapshot invalidates a previously computed path, without automatic replanning.
      if (path_valid_) {
        clear_path();
      }
    } catch (const std::exception & error) {
      grid_.reset();
      fail(PlanStatus::MAP_UNAVAILABLE, error.what());
    }
  }

  void on_goal(const geometry_msgs::msg::PoseStamped & input)
  {
    if (!grid_) {
      fail(PlanStatus::MAP_UNAVAILABLE, "No valid Phase 2 inflated map");
      return;
    }
    if (input.header.frame_id.empty() || !finite_pose(input.pose)) {
      fail(PlanStatus::INVALID_GOAL, "Goal requires a frame, finite pose and unit quaternion");
      return;
    }
    try {
      const auto goal = input.header.frame_id == planning_frame_ ? input :
        buffer_->transform(input, planning_frame_, tf2::durationFromSec(tf_timeout_));
      const auto start_tf = buffer_->lookupTransform(
        planning_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_timeout_));
      if (start_tf.header.stamp.sec != 0 || start_tf.header.stamp.nanosec != 0) {
        const auto stamp = rclcpp::Time(start_tf.header.stamp, get_clock()->get_clock_type());
        if (std::abs((now() - stamp).seconds()) > tf_max_age_) {
          throw tf2::ExtrapolationException("Current robot TF is stale or in the future");
        }
      }
      Cell start_cell{}, goal_cell{};
      if (!grid_->world_to_cell(
          start_tf.transform.translation.x, start_tf.transform.translation.y,
          start_cell.x, start_cell.y))
      {
        fail(PlanStatus::INVALID_START, "TF robot position is outside map or not finite");
        return;
      }
      if (!finite_pose(goal.pose) || !grid_->world_to_cell(
          goal.pose.position.x, goal.pose.position.y, goal_cell.x, goal_cell.y))
      {
        fail(PlanStatus::INVALID_GOAL, "Transformed goal is outside map or not finite");
        return;
      }
      const auto result = astar_search(grid_.get(), start_cell, goal_cell);
      if (result.status != PlanStatus::SUCCESS) {
        fail(result.status, "A* did not produce a valid path");
        return;
      }
      nav_msgs::msg::Path path;
      path.header.frame_id = planning_frame_;
      path.header.stamp = now();
      for (std::size_t i = 0; i < result.cells.size(); ++i) {
        const auto & cell = result.cells[i];
        const auto point = grid_->cell_to_world(cell.x, cell.y);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = point.first;
        pose.pose.position.y = point.second;
        pose.pose.position.z = map_z_;
        double yaw = tf2::getYaw(goal.pose.orientation);
        if (i + 1 < result.cells.size()) {
          const auto & next = result.cells[i + 1];
          const auto next_point = grid_->cell_to_world(next.x, next.y);
          yaw = std::atan2(next_point.second - point.second, next_point.first - point.first);
        }
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        path.poses.push_back(pose);
      }
      path_pub_->publish(path);
      path_valid_ = true;
      publish_status(
        PlanStatus::SUCCESS, "Published " + std::to_string(path.poses.size()) +
        " cell centres; cost=" + std::to_string(result.total_cost) + " m");
    } catch (const tf2::TransformException & error) {
      fail(PlanStatus::TF_UNAVAILABLE, error.what());
    } catch (const std::exception & error) {
      fail(PlanStatus::NO_PATH, error.what());
    }
  }

  std::string planning_frame_;
  std::string base_frame_;
  double tf_timeout_;
  double tf_max_age_;
  std::size_t max_cells_;
  double map_z_ = 0.0;
  bool path_valid_ = false;
  std::unique_ptr<PlanningGrid> grid_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};
}  // namespace plan

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<plan::AStarNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("planner"), "Startup failed: %s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}

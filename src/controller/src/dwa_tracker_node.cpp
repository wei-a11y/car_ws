// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include "controller/dwa_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace controller
{

class DwaTrackerNode : public rclcpp::Node
{
public:
  DwaTrackerNode()
  : Node("dwa_controller")
  {
    frame_ =
      declare_parameter<std::string>(
      "planning_frame", "map");

    base_ =
      declare_parameter<std::string>(
      "base_frame", "base_footprint");

    const double control_period =
      declare_parameter<double>(
      "control_period", 0.05);

    tf_timeout_sec_ =
      declare_parameter<double>(
      "tf_timeout_sec", 0.02);

    goal_tolerance_ =
      declare_parameter<double>(
      "goal_tolerance", 0.15);

    max_path_points_ =
      static_cast<std::size_t>(
      declare_parameter<int>(
        "max_path_points", 100000));

    /*
     * DWA参数
     * 这里全部对应你原来的代码。
     */
    DwaConfig config;

    config.max_linear =
      declare_parameter<double>(
      "max_linear", 0.12);

    config.min_linear =
      declare_parameter<double>(
      "min_linear", 0.03);

    config.max_angular =
      declare_parameter<double>(
      "max_angular", 0.6);

    config.sim_time =
      declare_parameter<double>(
      "sim_time", 1.0);

    config.sim_dt =
      declare_parameter<double>(
      "sim_dt", 0.05);

    config.heading_weight =
      declare_parameter<double>(
      "heading_weight", 1.2);

    config.dist_weight =
      declare_parameter<double>(
      "dist_weight", 1.0);

    config.vel_weight =
      declare_parameter<double>(
      "vel_weight", 0.5);

    config.path_weight =
      declare_parameter<double>(
      "path_weight", 0.8);

    config.obs_weight =
      declare_parameter<double>(
      "obs_weight", 0.5);

    config.collision_penalty =
      declare_parameter<double>(
      "collision_penalty", 50.0);

    config.min_lookahead =
      declare_parameter<double>(
      "min_lookahead", 0.15);

    config.max_lookahead =
      declare_parameter<double>(
      "max_lookahead", 0.40);

    config.lookahead_ratio =
      declare_parameter<double>(
      "lookahead_ratio", 0.50);

    config.v_samples =
      declare_parameter<int>(
      "v_samples", 10);

    config.w_samples =
      declare_parameter<int>(
      "w_samples", 20);

    config.path_backward_points =
      declare_parameter<int>(
      "path_backward_points", 10);

    config.path_forward_points =
      declare_parameter<int>(
      "path_forward_points", 50);

    config.obstacle_check_radius_cells =
      declare_parameter<int>(
      "obstacle_check_radius_cells", 1);

    config.occupied_threshold =
      declare_parameter<int>(
      "occupied_threshold", 50);

    config.max_path_points =
      max_path_points_;

    tracker_ =
      std::make_unique<DwaTracker>(config);

    /*
     * 输出：
     *
     * DWA -> /cmd_vel_raw
     * Safety Gate -> /cmd_vel
     */
    cmd_pub_ =
      create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw",
      rclcpp::QoS(1).reliable());

    /*
     * Phase 4输出的最终路径。
     */
    path_sub_ =
      create_subscription<nav_msgs::msg::Path>(
      "/plan",
      rclcpp::QoS(1).reliable(),
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        on_path(*msg);
      });

    /*
     * Phase 2输出的膨胀栅格。
     *
     * 注意：
     * 这里不是重新做A*，
     * 也不是重新膨胀地图。
     *
     * DWA直接利用项目现成的 inflated_grid。
     */
    grid_sub_ =
      create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/plan/inflated_grid",
      rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        on_grid(*msg);
      });

    /*
     * 保留你原来的 /goal_pose。
     *
     * DWA需要用它计算当前到终点距离，
     * 从而决定lookahead。
     */
    goal_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      rclcpp::QoS(10),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        on_goal(*msg);
      });

    buffer_ =
      std::make_unique<tf2_ros::Buffer>(
      get_clock());

    listener_ =
      std::make_unique<tf2_ros::TransformListener>(
      *buffer_,
      this,
      true);

    if (frame_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "planning_frame is empty. "
        "Please launch with planning_frame:=<actual map frame>");
    }

    timer_ =
      create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(control_period)),
      [this]() {
        on_timer();
      });

    RCLCPP_INFO(
      get_logger(),
      "DWA controller started: "
      "path=/plan grid=/plan/inflated_grid raw_cmd=/cmd_vel_raw");
  }

private:

  void publish_stop()
  {
    geometry_msgs::msg::Twist cmd;

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    cmd_pub_->publish(cmd);
  }
  bool transform_pose_to_planning_frame(
      geometry_msgs::msg::PoseStamped input,
      const std::string & fallback_frame,
      geometry_msgs::msg::PoseStamped & output,
      const char * label)
  {
      if (frame_.empty()) {
          RCLCPP_ERROR(
              get_logger(),
              "planning_frame is empty; cannot transform %s",
              label);
          return false;
      }

      // 单个 PoseStamped 没有 frame_id 时，使用上层消息的 frame_id
      if (input.header.frame_id.empty()) {
          input.header.frame_id = fallback_frame;
      }

      if (input.header.frame_id.empty()) {
          RCLCPP_ERROR(
              get_logger(),
              "%s has an empty frame_id",
              label);
          return false;
      }

      // 已经处于规划坐标系，不需要转换
      if (input.header.frame_id == frame_) {
          output = input;
          return true;
      }

      // 转换到 DWA 使用的规划坐标系
      try {
          output = buffer_->transform(
              input,
              frame_,
              tf2::durationFromSec(tf_timeout_sec_));

          return true;
      } catch (const tf2::TransformException & ex) {
          RCLCPP_ERROR(
              get_logger(),
              "Failed to transform %s from '%s' to '%s': %s",
              label,
              input.header.frame_id.c_str(),
              frame_.c_str(),
              ex.what());

          return false;
      }
  }

  bool get_robot_pose(
    double & x,
    double & y,
    double & yaw)
  {
    if (frame_.empty()) {
      return false;
    }

    try {
      const auto transform =
        buffer_->lookupTransform(
        frame_,
        base_,
        tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));

    x = transform.transform.translation.x;
    y = transform.transform.translation.y;

    const auto & q = transform.transform.rotation;

    yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );

      return std::isfinite(x) &&
             std::isfinite(y) &&
             std::isfinite(yaw);

    } catch (const tf2::TransformException & ex) {

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "TF lookup failed: %s",
        ex.what());

      return false;
    }
  }


  void on_path(const nav_msgs::msg::Path & msg)
{
    if (msg.poses.empty()) {
        tracker_->clear_path();

        RCLCPP_WARN(
            get_logger(),
            "Received empty /plan");

        return;
    }

    if (msg.poses.size() > max_path_points_) {
        tracker_->clear_path();

        RCLCPP_ERROR(
            get_logger(),
            "Path has too many points: %zu",
            msg.poses.size());

        return;
    }

    if (msg.header.frame_id.empty()) {
        tracker_->clear_path();

        RCLCPP_ERROR(
            get_logger(),
            "Received /plan with empty header.frame_id");

        return;
    }

    std::vector<DwaPoint> path;
    path.reserve(msg.poses.size());

    for (const auto & pose : msg.poses) {
        geometry_msgs::msg::PoseStamped source_pose = pose;

        // 如果单个路径点没有 frame_id，使用 Path 的 frame_id
        if (source_pose.header.frame_id.empty()) {
            source_pose.header.frame_id = msg.header.frame_id;
        }

        geometry_msgs::msg::PoseStamped transformed_pose;

        if (!transform_pose_to_planning_frame(
                source_pose,
                msg.header.frame_id,
                transformed_pose,
                "/plan point")) {
            tracker_->clear_path();
            return;
        }

        const double x =
            transformed_pose.pose.position.x;

        const double y =
            transformed_pose.pose.position.y;

        if (!std::isfinite(x) || !std::isfinite(y)) {
            tracker_->clear_path();

            RCLCPP_ERROR(
                get_logger(),
                "Received non-finite point in /plan");

            return;
        }

        path.push_back({x, y});
    }

    try {
        tracker_->set_path(path);
    } catch (const std::exception & ex) {
        tracker_->clear_path();

        RCLCPP_ERROR(
            get_logger(),
            "Failed to set DWA path: %s",
            ex.what());

        return;
    }

    RCLCPP_INFO(
        get_logger(),
        "DWA received /plan: %zu points; source_frame='%s', "
        "planning_frame='%s'",
        path.size(),
        msg.header.frame_id.c_str(),
        frame_.c_str());
}


  void on_grid(
    const nav_msgs::msg::OccupancyGrid & msg)
  {
    DwaGrid grid;
    if (msg.header.frame_id.empty()) {
    tracker_->clear_grid();

    RCLCPP_ERROR(
        get_logger(),
        "Received inflated grid with empty frame_id");

    return;
}

geometry_msgs::msg::PoseStamped grid_origin;

grid_origin.header = msg.header;
grid_origin.pose = msg.info.origin;

geometry_msgs::msg::PoseStamped origin_in_planning_frame;

if (!transform_pose_to_planning_frame(
        grid_origin,
        msg.header.frame_id,
        origin_in_planning_frame,
        "/plan/inflated_grid origin")) {
    tracker_->clear_grid();
    return;
}
    grid.width = msg.info.width;
    grid.height = msg.info.height;
    grid.resolution = msg.info.resolution;

    grid.origin_x =
        origin_in_planning_frame.pose.position.x;

    grid.origin_y =
        origin_in_planning_frame.pose.position.y;

    const auto & q =
        origin_in_planning_frame.pose.orientation;

    grid.origin_yaw =
      std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
      );

    grid.data = msg.data;

    try {
      tracker_->set_grid(grid);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid inflated grid: %s",
        ex.what());

      tracker_->clear_grid();
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "DWA received inflated grid: %ux%u res=%.3f",
      msg.info.width,
      msg.info.height,
      msg.info.resolution);
  }


void on_goal(const geometry_msgs::msg::PoseStamped & msg)
{
    geometry_msgs::msg::PoseStamped goal_in_planning_frame;

    if (!transform_pose_to_planning_frame(
            msg,
            "",
            goal_in_planning_frame,
            "/goal_pose")) {
        has_goal_ = false;
        tracker_->clear_goal();
        return;
    }

    goal_x_ =
        goal_in_planning_frame.pose.position.x;

    goal_y_ =
        goal_in_planning_frame.pose.position.y;

    if (!std::isfinite(goal_x_) || !std::isfinite(goal_y_)) {
        has_goal_ = false;
        tracker_->clear_goal();

        RCLCPP_ERROR(
            get_logger(),
            "Received non-finite /goal_pose");

        return;
    }

    has_goal_ = true;

    tracker_->set_goal({goal_x_, goal_y_});

    RCLCPP_INFO(
        get_logger(),
        "DWA received goal in '%s': %.3f %.3f",
        frame_.c_str(),
        goal_x_,
        goal_y_);
}


  void on_timer()
  {
    /*
     * 没有路径：
     * 停车。
     */
    if (!tracker_->has_path()) {
      publish_stop();
      return;
    }

    /*
     * 没有地图：
     * DWA无法进行障碍检查。
     */
    if (!tracker_->has_grid()) {
      publish_stop();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "DWA waiting for /plan/inflated_grid");

      return;
    }

    double rx;
    double ry;
    double ryaw;

    if (!get_robot_pose(rx, ry, ryaw)) {
      publish_stop();
      return;
    }

    /*
     * 如果收到goal_pose，
     * 使用真正的目标计算dist_to_goal。
     *
     * 如果没有goal，
     * 使用/plan最后一个点。
     */
    double target_x;
    double target_y;

    if (!has_goal_) {
      publish_stop();

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "DWA waiting for /goal_pose");

      return;
    }

    target_x = goal_x_;
    target_y = goal_y_;

    const double dist_to_goal =
      std::hypot(
      target_x - rx,
      target_y - ry);

    /*
     * 到目标点以后停车。
     */
    if (dist_to_goal <= goal_tolerance_) {
      publish_stop();

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "DWA reached goal: %.3f m",
        dist_to_goal);

      return;
    }

    const DwaPose pose{
      rx,
      ry,
      ryaw
    };

    const DwaCommand command =
      tracker_->compute(
      pose,
      dist_to_goal);

    geometry_msgs::msg::Twist msg;

    msg.linear.x = command.v;
    msg.angular.z = command.w;

    cmd_pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "DWA: pose=(%.2f, %.2f, %.2f) "
      "goal_dist=%.2f path=%zu "
      "v=%.3f w=%.3f",
      rx,
      ry,
      ryaw,
      dist_to_goal,
      tracker_->path_size(),
      command.v,
      command.w);
  }


private:

  std::string frame_;
  std::string base_;

  double tf_timeout_sec_;
  double goal_tolerance_;

  std::size_t max_path_points_;

  double goal_x_ = 0.0;
  double goal_y_ = 0.0;

  bool has_goal_ = false;

  std::unique_ptr<DwaTracker> tracker_;

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace controller


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(
      std::make_shared<controller::DwaTrackerNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("dwa_controller"),
      "%s",
      ex.what());

    rclcpp::shutdown();

    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
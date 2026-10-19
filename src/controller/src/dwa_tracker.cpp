// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0

#include "controller/dwa_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace controller
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

bool finite_point(const DwaPoint & p)
{
  return std::isfinite(p.x) && std::isfinite(p.y);
}

double point_distance(const DwaPoint & a, const DwaPoint & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

}  // namespace

double point_to_segment_distance(
    const DwaPoint & p,
    const DwaPoint & a,
    const DwaPoint & b)
{
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;

    const double apx = p.x - a.x;
    const double apy = p.y - a.y;

    const double ab_squared =
        abx * abx + aby * aby;

    // 线段退化为一个点
    if (ab_squared < 1e-12) {
        return std::hypot(
            p.x - a.x,
            p.y - a.y);
    }

    // 计算 p 在线段 ab 上的投影比例
    const double t = std::clamp(
        (apx * abx + apy * aby) / ab_squared,
        0.0,
        1.0);

    const double nearest_x = a.x + t * abx;
    const double nearest_y = a.y + t * aby;

    return std::hypot(
        p.x - nearest_x,
        p.y - nearest_y);
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }

  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }

  return angle;
}


void validate_config(const DwaConfig & config)
{
  if (!std::isfinite(config.max_linear) ||
    !std::isfinite(config.min_linear) ||
    !std::isfinite(config.max_angular) ||
    !std::isfinite(config.sim_time) ||
    !std::isfinite(config.sim_dt))
  {
    throw std::invalid_argument("DWA numeric parameters must be finite");
  }

  if (config.max_linear <= 0.0 ||
    config.min_linear < 0.0 ||
    config.max_linear < config.min_linear ||
    config.max_angular <= 0.0)
  {
    throw std::invalid_argument("Invalid DWA velocity limits");
  }

  if (config.sim_time <= 0.0 || config.sim_dt <= 0.0) {
    throw std::invalid_argument("Invalid DWA simulation parameters");
  }

  if (config.heading_weight < 0.0 ||
    config.dist_weight < 0.0 ||
    config.vel_weight < 0.0 ||
    config.path_weight < 0.0 ||
    config.obs_weight < 0.0 ||
    config.collision_penalty < 0.0)
  {
    throw std::invalid_argument("DWA weights must be nonnegative");
  }

  if (config.min_lookahead <= 0.0 ||
    config.max_lookahead < config.min_lookahead ||
    config.lookahead_ratio <= 0.0)
  {
    throw std::invalid_argument("Invalid lookahead parameters");
  }

  if (config.v_samples <= 0 || config.w_samples <= 0) {
    throw std::invalid_argument("DWA sample counts must be positive");
  }

  if (config.path_backward_points < 0 ||
    config.path_forward_points < 0)
  {
    throw std::invalid_argument("Invalid path search window");
  }

  if (config.obstacle_check_radius_cells < 0) {
    throw std::invalid_argument("Invalid obstacle check radius");
  }

  if (config.occupied_threshold < 0 ||
    config.occupied_threshold > 100)
  {
    throw std::invalid_argument("Invalid occupancy threshold");
  }

  if (config.max_path_points == 0) {
    throw std::invalid_argument("max_path_points must be positive");
  }
}


std::tuple<double, double, double> predict_pose(
  double x,
  double y,
  double yaw,
  double v,
  double w,
  double dt)
{
  x += v * std::cos(yaw) * dt;
  y += v * std::sin(yaw) * dt;
  yaw += w * dt;

  return {x, y, yaw};
}


DwaTracker::DwaTracker(DwaConfig config)
: config_(config)
{
  validate_config(config_);
}


void DwaTracker::clear_path()
{
  path_.clear();
  last_closest_idx_ = 0;
}


void DwaTracker::set_path(const std::vector<DwaPoint> & path)
{
  if (path.size() > config_.max_path_points) {
    throw std::invalid_argument("Path exceeds max_path_points");
  }

  path_.clear();
  path_.reserve(path.size());

  for (const auto & point : path) {
    if (!finite_point(point)) {
      throw std::invalid_argument("Path contains non-finite point");
    }

    if (!path_.empty()) {
      if (point.x == path_.back().x &&
        point.y == path_.back().y)
      {
        continue;
      }
    }

    path_.push_back(point);
  }

  last_closest_idx_ = 0;
}


bool DwaTracker::has_path() const
{
  return !path_.empty();
}


void DwaTracker::set_grid(const DwaGrid & grid)
{
  if (grid.width == 0 ||
    grid.height == 0 ||
    grid.data.size() != grid.width * grid.height ||
    !std::isfinite(grid.resolution) ||
    grid.resolution <= 0.0 ||
    !std::isfinite(grid.origin_x) ||
    !std::isfinite(grid.origin_y) ||
    !std::isfinite(grid.origin_yaw))
  {
    throw std::invalid_argument("Invalid DWA occupancy grid");
  }

  grid_ = grid;
  has_grid_ = true;
}


void DwaTracker::clear_grid()
{
  grid_ = DwaGrid{};
  has_grid_ = false;
}


bool DwaTracker::has_grid() const
{
  return has_grid_;
}


void DwaTracker::set_goal(DwaPoint goal)
{
  if (!finite_point(goal)) {
    throw std::invalid_argument("Invalid DWA goal");
  }

  goal_ = goal;
  has_goal_ = true;
}


void DwaTracker::clear_goal()
{
  has_goal_ = false;
}


bool DwaTracker::has_goal() const
{
  return has_goal_;
}


std::size_t DwaTracker::path_size() const
{
  return path_.size();
}


int DwaTracker::last_closest_index() const
{
  return last_closest_idx_;
}


int DwaTracker::find_closest_path_index(
  double rx,
  double ry)
{
  if (path_.empty()) {
    return 0;
  }

  const int start =
    std::max(0, last_closest_idx_ - 5);

  int closest_idx = start;

  double min_dist =
    std::numeric_limits<double>::max();

  for (std::size_t i = static_cast<std::size_t>(start);
    i < path_.size(); ++i)
  {
    const double dx = path_[i].x - rx;
    const double dy = path_[i].y - ry;

    const double d = std::hypot(dx, dy);

    if (d < min_dist) {
      min_dist = d;
      closest_idx = static_cast<int>(i);
    }
  }

  return closest_idx;
}


DwaPoint DwaTracker::get_lookahead_point(
  double rx,
  double ry,
  int closest_idx,
  double dist_to_goal)
{
  // 完全对应你的原代码：
  //
  // double lookahead_dist =
  //     std::min(0.4,
  //       std::max(0.15,
  //         dist_to_goal * 0.5));

  const double lookahead_dist =
    std::min(
      config_.max_lookahead,
      std::max(
        config_.min_lookahead,
        dist_to_goal * config_.lookahead_ratio));

  for (std::size_t i = static_cast<std::size_t>(closest_idx);
    i < path_.size(); ++i)
  {
    const double dx = path_[i].x - rx;
    const double dy = path_[i].y - ry;

    if (std::hypot(dx, dy) >= lookahead_dist) {
      return path_[i];
    }
  }

  return path_.back();
}


bool DwaTracker::world_to_grid(
  double wx,
  double wy,
  int & gx,
  int & gy) const
{
  if (!has_grid_) {
    return false;
  }

  const double dx = wx - grid_.origin_x;
  const double dy = wy - grid_.origin_y;

  const double c = std::cos(grid_.origin_yaw);
  const double s = std::sin(grid_.origin_yaw);

  const double local_x =
    c * dx + s * dy;

  const double local_y =
    -s * dx + c * dy;

  gx = static_cast<int>(
    std::floor(local_x / grid_.resolution));

  gy = static_cast<int>(
    std::floor(local_y / grid_.resolution));

  return gx >= 0 &&
    gy >= 0 &&
    gx < static_cast<int>(grid_.width) &&
    gy < static_cast<int>(grid_.height);
}


bool DwaTracker::grid_cell_blocked(
  int gx,
  int gy) const
{
  if (!has_grid_) {
    return true;
  }

  if (gx < 0 ||
    gy < 0 ||
    gx >= static_cast<int>(grid_.width) ||
    gy >= static_cast<int>(grid_.height))
  {
    return true;
  }

  const auto value =
    grid_.data[
      static_cast<std::size_t>(gy) * grid_.width +
      static_cast<std::size_t>(gx)];

  // 你的原算法：
  //
  // if (val < 0 || val > 50)
  //
  // 都认为碰撞。
  return value < 0 ||
    value > config_.occupied_threshold;
}


bool DwaTracker::trajectory_collision(
  double x,
  double y,
  bool & collision,
  double & obs_cost) const
{
  int gx;
  int gy;

  if (!world_to_grid(x, y, gx, gy)) {
    collision = true;
    obs_cost += 1.0;
    return true;
  }

  for (int ddy = -config_.obstacle_check_radius_cells;
    ddy <= config_.obstacle_check_radius_cells; ++ddy)
  {
    for (int ddx = -config_.obstacle_check_radius_cells;
      ddx <= config_.obstacle_check_radius_cells; ++ddx)
    {
      const int nx = gx + ddx;
      const int ny = gy + ddy;

      if (nx < 0 ||
        ny < 0 ||
        nx >= static_cast<int>(grid_.width) ||
        ny >= static_cast<int>(grid_.height))
      {
        continue;
      }

      const auto value =
        grid_.data[
          static_cast<std::size_t>(ny) * grid_.width +
          static_cast<std::size_t>(nx)];

      if (value < 0 ||
        value > config_.occupied_threshold)
      {
        collision = true;
        obs_cost += 1.0;
      }
    }
  }

  return collision;
}


DwaCommand DwaTracker::compute(
  const DwaPose & pose,
  double dist_to_goal)
{
  DwaCommand best_cmd{
    0.0,
    0.0
  };

  if (path_.empty()) {
    return best_cmd;
  }

  if (!has_grid_) {
    return best_cmd;
  }

  const int closest_idx =
    find_closest_path_index(
      pose.x,
      pose.y);

  last_closest_idx_ = closest_idx;

  const DwaPoint target =
    get_lookahead_point(
      pose.x,
      pose.y,
      closest_idx,
      dist_to_goal);

  const double target_x = target.x;
  const double target_y = target.y;

  // 计算当前机器人朝向与前视目标点方向之间的误差
  const double target_heading = std::atan2(
        target_y - pose.y,
        target_x - pose.x
  );

  const double heading_error_to_target = normalize_angle(
        target_heading - pose.yaw
  );

  // 朝向误差超过约 34 度时，优先原地旋转
  constexpr double kRotateInPlaceThreshold = 0.60;

  const bool rotate_in_place =
      std::abs(heading_error_to_target) > kRotateInPlaceThreshold;

    double best_score =
      -std::numeric_limits<double>::max();
      int valid_count = 0;
      int collision_count = 0;

  const int lo =
    std::max(
      0,
      closest_idx -
      config_.path_backward_points);

  const int hi =
    std::min(
      static_cast<int>(path_.size()) - 1,
      closest_idx +
      config_.path_forward_points);

  /*
   * 保持你的原采样逻辑：
   *
   * v:
   * [min_linear, max_linear]
   *
   * w:
   * [-max_angular, max_angular]
   */
  
  // 朝向误差较大时，只采样 v=0，避免一边前进一边大角度转向
  const int v_intervals =
      rotate_in_place ? 0 : config_.v_samples;

  for (int vi = 0; vi <= v_intervals; ++vi)
  {
      double v = 0.0;

      if (rotate_in_place)
      {
          // 原地旋转模式：线速度固定为 0
          v = 0.0;
      }
      else
      {
          // 正常跟踪模式：在最小和最大线速度之间采样
          v = config_.min_linear +
              (config_.max_linear - config_.min_linear) *
              static_cast<double>(vi) /
              static_cast<double>(config_.v_samples);
      }

      // 角速度仍然在 [-max_angular, max_angular] 内采样
      for (int wi = 0; wi <= config_.w_samples; ++wi)
      {
          const double w =
              -config_.max_angular +
              2.0 * config_.max_angular *
              static_cast<double>(wi) /
              static_cast<double>(config_.w_samples);


      double sim_x = pose.x;
      double sim_y = pose.y;
      double sim_yaw = pose.yaw;

      double path_cost = 0.0;
      double obs_cost = 0.0;

      bool collision = false;

      /*
       * 你的原代码：
       *
       * for (double t = 0;
       *      t < sim_time_;
       *      t += sim_dt_)
       */
      for (
        double t = 0.0;
        t < config_.sim_time;
        t += config_.sim_dt)
      {
        std::tie(
          sim_x,
          sim_y,
          sim_yaw) =
          predict_pose(
            sim_x,
            sim_y,
            sim_yaw,
            v,
            w,
            config_.sim_dt);

        /*
         * 轨迹点到全局路径的最小距离。
         *
         * 完全保留你原来的评分方式。
         */
      const DwaPoint predicted_point{
          sim_x,
          sim_y
      };

      double min_path_dist =
          std::numeric_limits<double>::max();

      if (lo == hi) {
          // 当前搜索窗口只有一个点
          min_path_dist = point_distance(
              predicted_point,
              path_[static_cast<std::size_t>(lo)]);
      } else {
          // 计算预测位置到路径线段的最小距离
          for (int i = lo; i < hi; ++i) {
              const double d = point_to_segment_distance(
                  predicted_point,
                  path_[static_cast<std::size_t>(i)],
                  path_[static_cast<std::size_t>(i + 1)]);

              if (d < min_path_dist) {
                  min_path_dist = d;
              }
          }
      }

      path_cost += min_path_dist;

        /*
         * 障碍物检查。
         *
         * 仍然是不立即淘汰，而是累计 obs_cost。
         */
        trajectory_collision(
          sim_x,
          sim_y,
          collision,
          obs_cost);
      }

      const double dx =
        target_x - sim_x;

      const double dy =
        target_y - sim_y;

      const double dist =
        std::hypot(dx, dy);

      const double target_angle =
        std::atan2(dy, dx);

      const double heading_err =
        std::abs(
          normalize_angle(
            target_angle - sim_yaw));

      /*
       * 完全对应你的原公式：
       *
       * score =
       *     - heading_weight_ * heading_err
       *     - dist_weight_    * dist
       *     - path_weight_    * path_cost
       *     - obs_weight_     * obs_cost
       *     + vel_weight_     * v;
       */
      double score =
        -config_.heading_weight * heading_err
        -config_.dist_weight * dist
        -config_.path_weight * path_cost
        -config_.obs_weight * obs_cost
        +config_.vel_weight * v;

      if (collision) {
        score -= config_.collision_penalty;
        collision_count++;
      }

      valid_count++;

      if (score > best_score) {
        best_score = score;

        best_cmd.v = v;
        best_cmd.w = w;
      }
    }
  }

  return best_cmd;
}

}  // namespace controller
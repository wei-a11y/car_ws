// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef CONTROLLER__DWA_TRACKER_HPP_
#define CONTROLLER__DWA_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

namespace controller
{

struct DwaPoint
{
  double x;
  double y;
};

struct DwaPose
{
  double x;
  double y;
  double yaw;
};

struct DwaCommand
{
  double v;
  double w;
};

struct DwaGrid
{
  std::size_t width = 0;
  std::size_t height = 0;

  double resolution = 0.0;

  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;

  std::vector<std::int8_t> data;
};

struct DwaConfig
{
  // DWA参数，保持你原来的参数
  double max_linear = 0.12;
  double min_linear = 0.03;
  double max_angular = 0.6;

  double sim_time = 1.0;
  double sim_dt = 0.05;

  double heading_weight = 1.2;
  double dist_weight = 1.0;
  double vel_weight = 0.5;
  double path_weight = 0.8;
  double obs_weight = 0.5;
  double collision_penalty = 50.0;

  // 路径前视
  double min_lookahead = 0.15;
  double max_lookahead = 0.40;
  double lookahead_ratio = 0.50;

  // DWA采样
  int v_samples = 10;
  int w_samples = 20;

  // 路径搜索窗口
  int path_backward_points = 10;
  int path_forward_points = 50;

  // 障碍检查
  int obstacle_check_radius_cells = 1;

  // OccupancyGrid阈值
  int occupied_threshold = 50;

  // 最大路径点
  std::size_t max_path_points = 100000;
};

void validate_config(const DwaConfig & config);

double normalize_angle(double angle);

std::tuple<double, double, double> predict_pose(
  double x,
  double y,
  double yaw,
  double v,
  double w,
  double dt);

class DwaTracker
{
public:
  explicit DwaTracker(DwaConfig config);

  void clear_path();

  void set_path(const std::vector<DwaPoint> & path);

  bool has_path() const;

  void set_grid(const DwaGrid & grid);

  void clear_grid();

  bool has_grid() const;

  void set_goal(DwaPoint goal);

  void clear_goal();

  bool has_goal() const;

    DwaCommand compute(
    const DwaPose & pose,
    double dist_to_goal);

  std::size_t path_size() const;

  int last_closest_index() const;

private:
  int find_closest_path_index(double rx, double ry);

  DwaPoint get_lookahead_point(
    double rx,
    double ry,
    int closest_idx,
    double dist_to_goal);

  bool world_to_grid(
    double wx,
    double wy,
    int & gx,
    int & gy) const;

  bool grid_cell_blocked(int gx, int gy) const;

  bool trajectory_collision(
    double x,
    double y,
    bool & collision,
    double & obs_cost) const;

private:
  DwaConfig config_;

  std::vector<DwaPoint> path_;

  DwaGrid grid_;
  bool has_grid_ = false;

  DwaPoint goal_{0.0, 0.0};
  bool has_goal_ = false;

  int last_closest_idx_ = 0;
};

}  // namespace controller

#endif  // CONTROLLER__DWA_TRACKER_HPP_
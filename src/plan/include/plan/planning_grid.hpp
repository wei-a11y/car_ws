// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__PLANNING_GRID_HPP_
#define PLAN__PLANNING_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace plan
{
struct GridGeometry
{
  std::size_t width;
  std::size_t height;
  double resolution;
  double origin_x;
  double origin_y;
  double origin_yaw;
};

struct GridConfig
{
  int occupied_threshold;
  bool unknown_is_obstacle;
  double robot_radius;
  double safety_margin;
  bool inflate_map_boundary;
  std::size_t max_cells;
};

// ROS-independent snapshot. Cell indices are row-major: y * width + x.
class PlanningGrid
{
public:
  PlanningGrid(GridGeometry geometry, std::vector<std::int8_t> occupancy, GridConfig config);
  static void validate_config(const GridConfig & config);
  // Import Phase 2 output unchanged: 100 blocked, 0 free, -1 allowed unknown.
  static PlanningGrid from_inflated_grid(
    GridGeometry geometry, std::vector<std::int8_t> cells, std::size_t max_cells);

  const GridGeometry & geometry() const {return geometry_;}
  const std::vector<std::int8_t> & raw_grid() const {return raw_;}
  const std::vector<std::uint8_t> & raw_obstacles() const {return obstacles_;}
  const std::vector<std::int8_t> & inflated_grid() const {return inflated_;}
  bool is_blocked(std::size_t x, std::size_t y) const;
  std::pair<double, double> cell_to_world(std::size_t x, std::size_t y) const;
  bool world_to_cell(double x, double y, std::size_t & cell_x, std::size_t & cell_y) const;

private:
  PlanningGrid(GridGeometry geometry, std::vector<std::int8_t> cells, std::size_t max_cells);
  std::size_t index(std::size_t x, std::size_t y) const;
  GridGeometry geometry_;
  std::vector<std::int8_t> raw_;
  std::vector<std::uint8_t> obstacles_;
  std::vector<std::int8_t> inflated_;
};
}  // namespace plan
#endif  // PLAN__PLANNING_GRID_HPP_

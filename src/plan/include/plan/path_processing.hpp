// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__PATH_PROCESSING_HPP_
#define PLAN__PATH_PROCESSING_HPP_

#include <cstddef>
#include <vector>

#include "plan/planning_grid.hpp"

namespace plan
{
struct Point2D
{
  double x;
  double y;
};

struct PathPoint2D
{
  double x;
  double y;
  double yaw;
};

struct PathProcessingConfig
{
  bool enable_shortcut;
  double resample_spacing;  // metres along the polyline
  std::size_t shortcut_max_lookahead;  // bounds the greedy search work
  std::size_t max_output_points;
  bool enable_arc_smoothing = false;
  double arc_radius = 0.35;  // preferred radius in metres
  double arc_min_radius = 0.20;
  std::size_t arc_radius_samples = 6;  // includes preferred/space-limited and minimum radii
  double arc_max_deviation = 0.001;  // maximum arc-to-chord distance, metres
  double arc_max_angle = 0.10;  // maximum angle between arc samples, radians
};

struct PathProcessingStats
{
  std::size_t rounded_corners = 0;
  std::size_t retained_corners = 0;
};

void validate_path_processing_config(const PathProcessingConfig & config);
// Conservative supercover: blocked/outside cells touched at edges/corners also reject the line.
bool line_of_sight(const PlanningGrid & grid, Point2D from, Point2D to);
// Throws on invalid/colliding input or resource limits. Empty input produces empty output.
// Preserves endpoints. Optional tangent arcs replace only collision-validated bends;
// unsuitable bends remain available for stop/align tracking. No timed trajectory is produced.
// Arc vertices and retained bends are kept in addition to uniform polyline arc-length samples.
// Last yaw follows the final segment; a singleton uses yaw=0 (no tangent is available).
std::vector<PathPoint2D> process_path(
  const PlanningGrid & grid, const std::vector<Point2D> & raw,
  const PathProcessingConfig & config, PathProcessingStats * stats = nullptr);
}  // namespace plan
#endif  // PLAN__PATH_PROCESSING_HPP_

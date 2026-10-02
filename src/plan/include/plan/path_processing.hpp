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
};

void validate_path_processing_config(const PathProcessingConfig & config);
// Conservative supercover: blocked/outside cells touched at edges/corners also reject the line.
bool line_of_sight(const PlanningGrid & grid, Point2D from, Point2D to);
// Throws on invalid/colliding input or resource limits. Empty input produces empty output.
// Preserves endpoints and bends in addition to uniform arc-length samples: no corner chords.
// Last yaw follows the final segment; a singleton uses yaw=0 (no tangent is available).
std::vector<PathPoint2D> process_path(
  const PlanningGrid & grid, const std::vector<Point2D> & raw,
  const PathProcessingConfig & config);
}  // namespace plan
#endif  // PLAN__PATH_PROCESSING_HPP_

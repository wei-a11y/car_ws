// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PATH_OPTIMIZER_HPP_
#define PATH_OPTIMIZER_HPP_
#include "plan/path_processing.hpp"
namespace plan
{
std::vector<PathPoint2D> optimize_joint_path(
  const PlanningGrid & grid, const std::vector<Point2D> & raw,
  const std::vector<PathPoint2D> & baseline, const PathProcessingConfig & config,
  PathProcessingStats & stats);
}
#endif  // PATH_OPTIMIZER_HPP_

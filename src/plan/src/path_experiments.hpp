// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PATH_EXPERIMENTS_HPP_
#define PATH_EXPERIMENTS_HPP_
#include "plan/path_processing.hpp"
#include "plan/speed_envelope.hpp"
namespace plan
{
std::vector<SpeedSegment> path_speed_segments(const std::vector<PathPoint2D> & path);
double executable_time(const std::vector<PathPoint2D> & path, const MotionLimits & motion);
std::vector<PathPoint2D> process_experiments(
  const PlanningGrid & grid, const std::vector<PathPoint2D> & baseline,
  const PathProcessingConfig & config, PathProcessingStats & stats);
}  // namespace plan
#endif  // PATH_EXPERIMENTS_HPP_

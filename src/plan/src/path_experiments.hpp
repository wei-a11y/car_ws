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
std::size_t path_speed_events(
  const std::vector<PathPoint2D> & path,
  const PathProcessingConfig & config);
std::vector<PathPoint2D> process_experiments(
  const PlanningGrid & grid, const std::vector<PathPoint2D> & baseline,
  const PathProcessingConfig & config, PathProcessingStats & stats);
std::size_t speed_envelope_events(
  const SpeedEnvelope & envelope,
  const PathProcessingConfig & config);
std::vector<PathPoint2D> process_frozen_speed(
  const std::vector<PathPoint2D> & path, const PathProcessingConfig & config,
  PathProcessingStats & stats, double time_limit = std::numeric_limits<double>::infinity());
}  // namespace plan
#endif  // PATH_EXPERIMENTS_HPP_

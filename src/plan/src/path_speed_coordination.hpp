// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PATH_SPEED_COORDINATION_HPP_
#define PATH_SPEED_COORDINATION_HPP_
#include "plan/path_processing.hpp"
#include "plan/speed_envelope.hpp"
namespace plan
{
double envelope_interval_time(const SpeedEnvelope & envelope, double begin, double end);
double envelope_variation(const SpeedEnvelope & envelope);
void coordinate_bounded_speed(
  std::vector<PathPoint2D> & path, SpeedEnvelope & envelope, const PathProcessingConfig & config,
  double time_limit, std::string & report);
}  // namespace plan
#endif  // PATH_SPEED_COORDINATION_HPP_

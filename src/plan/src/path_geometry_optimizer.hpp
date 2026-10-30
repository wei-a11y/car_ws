// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PATH_GEOMETRY_OPTIMIZER_HPP_
#define PATH_GEOMETRY_OPTIMIZER_HPP_
#include "bezier_geometry.hpp"
#include "plan/path_processing.hpp"
namespace plan
{
bezier::Bezier geometry_policy_curve(const double * a, const double * b);
struct GeometryCertificate
{
  bool safe = false;
  std::string reason, failure;
  std::size_t used = 0;
  std::vector<PathPoint2D> points;
  std::vector<double> envelopes;
  std::vector<double> curve_parameter;
  std::vector<double> length_upper;
};
GeometryCertificate certify_geometry(
  const PlanningGrid & grid, const std::vector<bezier::Bezier> & curves,
  const std::vector<PathPoint2D> & reference, const PathProcessingConfig & config,
  bool seed_only = false, std::size_t checks_already_used = 0);
std::string geometry_fingerprint(const std::vector<PathPoint2D> & path);
std::vector<PathPoint2D> optimize_window_geometry(
  const PlanningGrid & grid, const std::vector<PathPoint2D> & baseline,
  const PathProcessingConfig & config, PathProcessingStats & stats);
}  // namespace plan
#endif  // PATH_GEOMETRY_OPTIMIZER_HPP_

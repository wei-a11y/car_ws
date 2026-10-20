// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__PATH_PROCESSING_HPP_
#define PLAN__PATH_PROCESSING_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "plan/motion_limits.hpp"
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
  double tangent_yaw = 0.0;
  double curvature = 0.0;  // outgoing primitive; signed 1/metre
  bool stop_required = false;
  std::uint8_t segment_type = 0;  // 0 line, 1 continuous transition, 2 shaped-arc fallback
  double speed_limit = -1.0;  // negative only in the legacy internal API
  double curvature_bound = 0.0, curvature_rate_bound = 0.0;  // outgoing segment certificates
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
  double arc_radius_max = 0.0;  // zero keeps the preferred radius as the upper bound
  double arc_radius_weight = 1.0;
  double arc_curvature_weight = 0.04;
  bool recover_shortcut_corners = true;
  double retained_corner_cost = 2.0;  // equivalent metres per required stop
  bool enable_curvature_transition = false;
  double transition_length = 0.20;  // preferred length of EACH clothoid
  double transition_min_length = 0.04;
  std::size_t transition_samples = 4;
  double max_curvature_rate = 30.0;  // abs(d curvature / ds), 1/m^2
  bool enable_joint_optimization = false;
  bool enable_local_relocation = true, enable_arc_fallback = true;
  MotionLimits motion;
  double max_length_ratio = 1.10, local_shift_max = 0.20, local_window_length = 2.0;
  double local_anchor_spacing = 0.25;
  std::size_t local_shift_samples = 4, local_max_variants = 24, local_beam_width = 4;
  std::size_t joint_max_candidates = 64, joint_max_labels = 64, joint_work_budget = 2000000;
  double fallback_deviation = 0.01, fallback_blend_min = 0.01, fallback_blend_max = 0.08;
  std::size_t fallback_blend_samples = 4, curve_max_depth = 20;
  double clearance_target = 0.20, clearance_weight = 0.5, smoothness_weight = 0.2;
};

struct CandidateDiagnostic
{
  std::uint8_t kind = 0;
  double radius = 0, transition = 0, blend = 0, trim_in = 0, trim_out = 0, speed = 0;
  std::string result;
};

struct CornerDiagnostic
{
  double x = 0.0, y = 0.0;
  std::size_t candidates = 0, space = 0, collision = 0, curvature = 0, speed = 0;
  std::size_t deviation = 0, conflicts = 0, length = 0;
  bool retained = false;
  std::size_t chosen = 0;
  std::vector<CandidateDiagnostic> trials;
};

struct PathProcessingStats
{
  std::size_t rounded_corners = 0;
  std::size_t retained_corners = 0;
  std::size_t space_limited_corners = 0;
  std::size_t collision_limited_corners = 0;
  bool shortcut_recovered = false;
  std::size_t arc_fallbacks = 0, variants_evaluated = 0, work_used = 0;
  bool relocated = false, budget_exhausted = false;
  std::string budget_reason;
  double baseline_length = 0.0, selected_length = 0.0, estimated_time = 0.0;
  double minimum_clearance = 0.0;
  std::vector<CornerDiagnostic> corners;
};

void validate_path_processing_config(const PathProcessingConfig & config);
// Conservative supercover: blocked/outside cells touched at edges/corners also reject the line.
bool line_of_sight(const PlanningGrid & grid, Point2D from, Point2D to);
bool curve_chord_is_free(const PlanningGrid & grid, Point2D from, Point2D to, double envelope);
std::string processing_report(const PathProcessingStats & stats);
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

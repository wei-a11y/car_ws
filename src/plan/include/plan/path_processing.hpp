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
  bool enable_window_bezier = false, enable_turn_speed_coordination = false;
  std::string window_geometry_policy = "legacy";
  bool window_detailed_diagnostics = false;
  bool window_include_coarse_seed = true;
  double window_seed_fit_tolerance = 0.005, window_geometry_min_gain = 0.01;
  double window_parent_budget_fraction = 0.5, window_split_min_length = 1.0;
  std::size_t window_split_max_depth = 2, window_split_min_events = 2;
  double window_merge_gap = 1.0, window_extension = 0.30, window_knot_spacing = 0.50;
  double window_max_deviation = 0.20, window_max_time_ratio = 1.10;
  double window_target_speed = 0.25, window_target_radius = 0.70;
  double window_transition_scale = 0.25;
  double window_solver_seconds = 0.25, window_total_seconds = 1.0;
  std::size_t window_iterations = 50, window_objective_samples = 8, window_max_knots = 64;
  std::size_t window_check_budget = 200000;
  std::vector<double> window_seed_scales{0.75, 1.0, 1.25};
  double window_curvature_weight = 1.0, window_rate_weight = 1.0;
  double window_join_weight = 0.20, window_deviation_weight = 0.25;
  double window_clearance_weight = 0.50, window_speed_weight = 2.0;
  double window_constraint_weight = 100.0, window_constraint_fraction = 0.80;
  double window_min_tangent_ratio = 0.05;
  double window_derivative_min_scale = 0.25, window_derivative_max_scale = 2.0;
  double window_heading_max_change = 1.5707963267948966;
  double coordination_min_gain = 0.005, coordination_max_gap = 1.0;
  double coordination_acceleration_threshold = 0.02, coordination_min_duration = 0.20;
  double coordination_acceleration_filter_tau = 0.15;
  std::size_t coordination_max_rounds = 20;
  bool coordination_enable_platforms = true;
  double coordination_platform_min_speed = 0.20, coordination_platform_min_length = 0.30;
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
  std::string grid_fingerprint;
  std::string window_report;  // validated JSON array, generated by the window core
  double frozen_baseline_time = 0.0;
  std::size_t speed_events_before = 0, speed_events_after = 0;
  bool dynamic_budget = false, speed_envelope_fallback = false;
  std::string geometry_policy, final_geometry_id, geometry_fallback_reason;
  std::string geometry_reference_s;
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

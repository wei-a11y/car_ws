// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PROCESSING_PARAMETERS_HPP_
#define PROCESSING_PARAMETERS_HPP_
#include <cstdint>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "plan/path_processing.hpp"
namespace plan
{
template<typename T>
T processing_parameter(rclcpp::Node & node, const std::string & name)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  return node.declare_parameter<T>(name, descriptor);
}
template<typename T>
T processing_parameter(rclcpp::Node & node, const std::string & name, const T & value)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only = true;
  return node.declare_parameter<T>(name, value, descriptor);
}
inline PathProcessingConfig load_processing_config(rclcpp::Node & node)
{
  PathProcessingConfig config{};
  const auto lookahead = processing_parameter<std::int64_t>(
    node,
    "post_processing.shortcut_max_lookahead");
  const auto output_limit = processing_parameter<std::int64_t>(
    node,
    "post_processing.max_output_points");
  if (lookahead <= 0 || output_limit <= 0) {
    throw std::invalid_argument("Path processing limits must be positive");
  }
  config = {
    processing_parameter<bool>(node, "post_processing.enable_shortcut"),
    processing_parameter<double>(node, "post_processing.resample_spacing"),
    static_cast<std::size_t>(lookahead), static_cast<std::size_t>(output_limit)};
  config.enable_arc_smoothing =
    processing_parameter<bool>(node, "post_processing.enable_arc_smoothing");
  config.arc_radius = processing_parameter<double>(node, "post_processing.arc_radius");
  config.arc_min_radius = processing_parameter<double>(node, "post_processing.arc_min_radius");
  const auto radius_samples = processing_parameter<std::int64_t>(
    node,
    "post_processing.arc_radius_samples");
  if (radius_samples < 2) {
    throw std::invalid_argument("Arc radius samples must be at least two");
  }
  config.arc_radius_samples = static_cast<std::size_t>(radius_samples);
  config.arc_max_deviation =
    processing_parameter<double>(node, "post_processing.arc_max_deviation");
  config.arc_max_angle = processing_parameter<double>(node, "post_processing.arc_max_angle");
  config.arc_radius_max = processing_parameter<double>(node, "post_processing.arc_radius_max");
  config.arc_radius_weight = processing_parameter<double>(
    node,
    "post_processing.arc_radius_weight");
  config.arc_curvature_weight =
    processing_parameter<double>(node, "post_processing.arc_curvature_weight");
  config.recover_shortcut_corners =
    processing_parameter<bool>(node, "post_processing.recover_shortcut_corners");
  config.retained_corner_cost =
    processing_parameter<double>(node, "post_processing.retained_corner_cost");
  config.enable_curvature_transition =
    processing_parameter<bool>(node, "post_processing.enable_curvature_transition");
  config.transition_length =
    processing_parameter<double>(node, "post_processing.transition_length");
  config.transition_min_length =
    processing_parameter<double>(node, "post_processing.transition_min_length");
  const auto transition_samples =
    processing_parameter<std::int64_t>(node, "post_processing.transition_samples");
  if (transition_samples < 2) {
    throw std::invalid_argument("Transition samples must be at least two");
  }
  config.transition_samples = static_cast<std::size_t>(transition_samples);
  config.max_curvature_rate =
    processing_parameter<double>(node, "post_processing.max_curvature_rate");
  auto & m = config.motion;
  m.v_nominal = processing_parameter<double>(node, "v_nominal");
  m.v_max = processing_parameter<double>(node, "v_max");
  m.w_max = processing_parameter<double>(node, "w_max");
  m.linear_acceleration = processing_parameter<double>(node, "linear_acceleration");
  m.linear_deceleration = processing_parameter<double>(node, "linear_deceleration");
  m.angular_acceleration = processing_parameter<double>(node, "angular_acceleration");
  m.lateral_acceleration = processing_parameter<double>(node, "lateral_acceleration");
  m.feedforward_angular_fraction =
    processing_parameter<double>(node, "feedforward_angular_fraction");
  m.angular_acceleration_fraction = processing_parameter<double>(
    node,
    "angular_acceleration_fraction");
  m.min_turn_speed = processing_parameter<double>(node, "min_turn_speed");
  const auto positive_count = [&node](const std::string & name, std::int64_t fallback = -1) {
      const auto value = fallback < 0 ? processing_parameter<std::int64_t>(node, name) :
        processing_parameter<std::int64_t>(node, name, fallback);
      if (value <= 0) {throw std::invalid_argument(name + " must be positive");}
      return static_cast<std::size_t>(value);
    };
  auto & c = config;
  c.enable_joint_optimization = processing_parameter<bool>(
    node,
    "post_processing.enable_joint_optimization");
  c.enable_local_relocation = processing_parameter<bool>(
    node,
    "post_processing.enable_local_relocation");
  c.enable_arc_fallback = processing_parameter<bool>(node, "post_processing.enable_arc_fallback");
  c.max_length_ratio = processing_parameter<double>(node, "post_processing.max_length_ratio");
  c.local_shift_max = processing_parameter<double>(node, "post_processing.local_shift_max");
  c.local_window_length = processing_parameter<double>(node, "post_processing.local_window_length");
  c.local_anchor_spacing =
    processing_parameter<double>(node, "post_processing.local_anchor_spacing");
  c.local_shift_samples = positive_count("post_processing.local_shift_samples");
  c.local_max_variants = positive_count("post_processing.local_max_variants");
  c.local_beam_width = positive_count("post_processing.local_beam_width");
  c.joint_max_candidates = positive_count("post_processing.joint_max_candidates");
  c.joint_max_labels = positive_count("post_processing.joint_max_labels");
  c.joint_work_budget = positive_count("post_processing.joint_work_budget");
  c.fallback_deviation = processing_parameter<double>(node, "post_processing.fallback_deviation");
  c.fallback_blend_min = processing_parameter<double>(node, "post_processing.fallback_blend_min");
  c.fallback_blend_max = processing_parameter<double>(node, "post_processing.fallback_blend_max");
  c.fallback_blend_samples = positive_count("post_processing.fallback_blend_samples");
  c.curve_max_depth = positive_count("post_processing.curve_max_depth");
  c.clearance_target = processing_parameter<double>(node, "post_processing.clearance_target");
  c.clearance_weight = processing_parameter<double>(node, "post_processing.clearance_weight");
  c.smoothness_weight = processing_parameter<double>(node, "post_processing.smoothness_weight");
  c.enable_window_bezier = processing_parameter<bool>(
    node, "post_processing.enable_window_bezier",
    c.enable_window_bezier);
  c.enable_turn_speed_coordination = processing_parameter<bool>(
    node,
    "post_processing.enable_turn_speed_coordination", c.enable_turn_speed_coordination);
  c.window_geometry_policy = processing_parameter<std::string>(
    node, "post_processing.window_geometry_policy", c.window_geometry_policy);
  c.window_detailed_diagnostics = processing_parameter<bool>(
    node, "post_processing.window_detailed_diagnostics", c.window_detailed_diagnostics);
  c.window_include_coarse_seed = processing_parameter<bool>(
    node, "post_processing.window_include_coarse_seed", c.window_include_coarse_seed);
  c.window_refine_objective = processing_parameter<bool>(
    node, "post_processing.window_refine_objective", c.window_refine_objective);
  c.window_prioritize_coarse_seed = processing_parameter<bool>(
    node, "post_processing.window_prioritize_coarse_seed", c.window_prioritize_coarse_seed);
  c.window_local_refinement = processing_parameter<bool>(
    node, "post_processing.window_local_refinement", c.window_local_refinement);
  const auto refine_events = processing_parameter<std::int64_t>(
    node, "post_processing.window_refine_min_events", c.window_refine_min_events);
  if (refine_events < 0) {throw std::invalid_argument("Invalid refinement event count");}
  c.window_refine_min_events = static_cast<std::size_t>(refine_events);
  c.window_refine_rate_weight = processing_parameter<double>(
    node, "post_processing.window_refine_rate_weight", c.window_refine_rate_weight);
  c.window_refine_min_curvature = processing_parameter<double>(
    node, "post_processing.window_refine_min_curvature", c.window_refine_min_curvature);
  c.window_coarse_budget_fraction = processing_parameter<double>(
    node, "post_processing.window_coarse_budget_fraction", c.window_coarse_budget_fraction);
  c.window_peak_weight = processing_parameter<double>(
    node, "post_processing.window_peak_weight", c.window_peak_weight);
  c.window_max_curvature_weight = processing_parameter<double>(
    node, "post_processing.window_max_curvature_weight", c.window_max_curvature_weight);
  c.window_seed_fit_tolerance = processing_parameter<double>(
    node, "post_processing.window_seed_fit_tolerance", c.window_seed_fit_tolerance);
  c.window_geometry_min_gain = processing_parameter<double>(
    node, "post_processing.window_geometry_min_gain", c.window_geometry_min_gain);
  c.window_parent_budget_fraction = processing_parameter<double>(
    node, "post_processing.window_parent_budget_fraction", c.window_parent_budget_fraction);
  c.window_split_min_length = processing_parameter<double>(
    node, "post_processing.window_split_min_length", c.window_split_min_length);
  const auto split_depth = processing_parameter<std::int64_t>(
    node, "post_processing.window_split_max_depth", c.window_split_max_depth);
  if (split_depth < 0 || split_depth > 8) {
    throw std::invalid_argument("Invalid window split depth");
  }
  c.window_split_max_depth = static_cast<std::size_t>(split_depth);
  c.window_split_min_events = positive_count(
    "post_processing.window_split_min_events", c.window_split_min_events);
  c.window_merge_gap = processing_parameter<double>(
    node, "post_processing.window_merge_gap",
    c.window_merge_gap);
  c.window_extension = processing_parameter<double>(
    node, "post_processing.window_extension",
    c.window_extension);
  c.window_knot_spacing = processing_parameter<double>(
    node, "post_processing.window_knot_spacing",
    c.window_knot_spacing);
  c.window_max_deviation =
    processing_parameter<double>(
    node, "post_processing.window_max_deviation",
    c.window_max_deviation);
  c.window_max_time_ratio = processing_parameter<double>(
    node,
    "post_processing.window_max_time_ratio", c.window_max_time_ratio);
  c.window_target_speed = processing_parameter<double>(
    node, "post_processing.window_target_speed",
    c.window_target_speed);
  c.window_target_radius =
    processing_parameter<double>(
    node, "post_processing.window_target_radius",
    c.window_target_radius);
  c.window_transition_scale = processing_parameter<double>(
    node,
    "post_processing.window_transition_scale", c.window_transition_scale);
  c.window_solver_seconds = processing_parameter<double>(
    node,
    "post_processing.window_solver_seconds", c.window_solver_seconds);
  c.window_total_seconds =
    processing_parameter<double>(
    node, "post_processing.window_total_seconds",
    c.window_total_seconds);
  c.window_iterations = positive_count(
    "post_processing.window_iterations",
    static_cast<std::int64_t>(c.window_iterations));
  c.window_objective_samples = positive_count(
    "post_processing.window_objective_samples",
    static_cast<std::int64_t>(c.window_objective_samples));
  c.window_max_knots =
    positive_count(
    "post_processing.window_max_knots",
    static_cast<std::int64_t>(c.window_max_knots));
  c.window_check_budget =
    positive_count(
    "post_processing.window_check_budget",
    static_cast<std::int64_t>(c.window_check_budget));
  c.window_seed_scales = processing_parameter<std::vector<double>>(
    node,
    "post_processing.window_seed_scales", c.window_seed_scales);
  c.window_curvature_weight = processing_parameter<double>(
    node,
    "post_processing.window_curvature_weight", c.window_curvature_weight);
  c.window_rate_weight = processing_parameter<double>(
    node, "post_processing.window_rate_weight",
    c.window_rate_weight);
  c.window_join_weight = processing_parameter<double>(
    node, "post_processing.window_join_weight",
    c.window_join_weight);
  c.window_deviation_weight = processing_parameter<double>(
    node,
    "post_processing.window_deviation_weight", c.window_deviation_weight);
  c.window_clearance_weight = processing_parameter<double>(
    node,
    "post_processing.window_clearance_weight", c.window_clearance_weight);
  c.window_derivative_min_scale = processing_parameter<double>(
    node,
    "post_processing.window_derivative_min_scale",
    c.window_derivative_min_scale);
  c.window_derivative_max_scale = processing_parameter<double>(
    node,
    "post_processing.window_derivative_max_scale",
    c.window_derivative_max_scale);
  c.window_heading_max_change = processing_parameter<double>(
    node,
    "post_processing.window_heading_max_change",
    c.window_heading_max_change);
  c.window_constraint_fraction = processing_parameter<double>(
    node,
    "post_processing.window_constraint_fraction", c.window_constraint_fraction);
  c.window_min_tangent_ratio = processing_parameter<double>(
    node,
    "post_processing.window_min_tangent_ratio", c.window_min_tangent_ratio);
  c.window_constraint_weight = processing_parameter<double>(
    node,
    "post_processing.window_constraint_weight", c.window_constraint_weight);
  c.window_speed_weight = processing_parameter<double>(
    node, "post_processing.window_speed_weight",
    c.window_speed_weight);
  c.coordination_min_gain = processing_parameter<double>(
    node,
    "post_processing.coordination_min_gain", c.coordination_min_gain);
  c.coordination_enable_platforms = processing_parameter<bool>(
    node,
    "post_processing.coordination_enable_platforms", c.coordination_enable_platforms);
  c.coordination_platform_min_speed = processing_parameter<double>(
    node,
    "post_processing.coordination_platform_min_speed", c.coordination_platform_min_speed);
  c.coordination_platform_min_length = processing_parameter<double>(
    node,
    "post_processing.coordination_platform_min_length", c.coordination_platform_min_length);
  c.coordination_max_rounds =
    positive_count(
    "post_processing.coordination_max_rounds",
    static_cast<std::int64_t>(c.coordination_max_rounds));
  c.coordination_max_gap =
    processing_parameter<double>(
    node, "post_processing.coordination_max_gap",
    c.coordination_max_gap);
  c.coordination_acceleration_threshold = processing_parameter<double>(
    node,
    "post_processing.coordination_acceleration_threshold", c.coordination_acceleration_threshold);
  c.coordination_acceleration_filter_tau = processing_parameter<double>(
    node,
    "post_processing.coordination_acceleration_filter_tau", c.coordination_acceleration_filter_tau);
  c.coordination_min_duration = processing_parameter<double>(
    node,
    "post_processing.coordination_min_duration", c.coordination_min_duration);
  m.enable_dynamic_angular_budget =
    processing_parameter<bool>(
    node, "enable_dynamic_angular_budget",
    m.enable_dynamic_angular_budget);
  m.speed_envelope_rounds =
    positive_count("speed_envelope_rounds", static_cast<std::int64_t>(m.speed_envelope_rounds));
  m.speed_search_iterations =
    positive_count(
    "speed_search_iterations",
    static_cast<std::int64_t>(m.speed_search_iterations));
  m.dynamic_budget_utilization = processing_parameter<double>(
    node, "dynamic_budget_utilization",
    m.dynamic_budget_utilization);
  m.speed_envelope_tolerance = processing_parameter<double>(
    node, "speed_envelope_tolerance",
    m.speed_envelope_tolerance);
  validate_path_processing_config(config);
  return config;
}
}  // namespace plan
#endif  // PROCESSING_PARAMETERS_HPP_

// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__MOTION_LIMITS_HPP_
#define PLAN__MOTION_LIMITS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace plan
{
// Shared, ROS-independent constraints. Node wrappers load the same YAML.
struct MotionLimits
{
  double v_nominal = 0.15, v_max = 0.20, w_max = 0.60;
  double linear_acceleration = 0.10, linear_deceleration = 0.15;
  double angular_acceleration = 1.0, lateral_acceleration = 0.04;
  double feedforward_angular_fraction = 0.75, angular_acceleration_fraction = 0.75;
  double min_turn_speed = 0.03;
  bool enable_dynamic_angular_budget = false;
  std::size_t speed_envelope_rounds = 20;
  std::size_t speed_search_iterations = 48;
  double speed_envelope_tolerance = 1e-4;
  double dynamic_budget_utilization = 0.95;
};

inline void validate_motion_limits(const MotionLimits & m)
{
  for (double v : {m.v_nominal, m.v_max, m.w_max, m.linear_acceleration,
      m.linear_deceleration, m.angular_acceleration, m.lateral_acceleration,
      m.feedforward_angular_fraction, m.angular_acceleration_fraction, m.min_turn_speed})
  {
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument("Motion limits must be finite and positive");
    }
  }
  if (m.min_turn_speed > m.v_nominal || m.v_nominal > m.v_max ||
    m.feedforward_angular_fraction >= 1.0 || m.angular_acceleration_fraction >= 1.0)
  {
    throw std::invalid_argument("Inconsistent shared motion constraints");
  }
  if (!std::isfinite(m.dynamic_budget_utilization) || m.dynamic_budget_utilization <= 0 ||
    m.dynamic_budget_utilization > 1 || m.speed_envelope_rounds == 0 ||
    m.speed_search_iterations == 0 ||
    !std::isfinite(m.speed_envelope_tolerance) ||
    m.speed_envelope_tolerance <= 0.0)
  {
    throw std::invalid_argument("Invalid speed envelope convergence limits");
  }
}

struct CurveLimits {double speed, acceleration, deceleration;};

inline CurveLimits curve_limits(const MotionLimits & m, double curvature, double rate)
{
  if (!std::isfinite(curvature) || curvature < 0.0 || !std::isfinite(rate) || rate < 0.0) {
    throw std::invalid_argument("Invalid curvature/rate bound");
  }
  CurveLimits result{m.v_max, m.linear_acceleration, m.linear_deceleration};
  const double budget = m.angular_acceleration_fraction * m.angular_acceleration /
    (m.enable_dynamic_angular_budget ? 1.0 / m.dynamic_budget_utilization : 2.0);
  if (curvature > 0.0) {
    result.speed = std::min(
      {result.speed,
        m.feedforward_angular_fraction * m.w_max / curvature,
        std::sqrt(m.lateral_acceleration / curvature)});
    result.acceleration = std::min(result.acceleration, budget / curvature);
    result.deceleration = std::min(result.deceleration, budget / curvature);
  }
  if (rate > 0.0) {
    result.speed = std::min(result.speed, std::sqrt(budget / rate));
  }
  return result;
}
}  // namespace plan
#endif  // PLAN__MOTION_LIMITS_HPP_

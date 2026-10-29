// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__SPEED_ENVELOPE_HPP_
#define PLAN__SPEED_ENVELOPE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include "plan/motion_limits.hpp"

namespace plan
{
struct SpeedSegment
{
  double length, curvature, rate, cap;
  bool stop_after = false;
};
struct SpeedEnvelope
{
  // Each input segment is split in two: zero endpoints still allow an interior peak.
  std::vector<double> distance, speed;
  double time = 0.0;
  bool conservative_fallback = false;
};

inline double angular_budget(const MotionLimits & m)
{
  return m.angular_acceleration_fraction * m.angular_acceleration;
}

// Reachable upper speed over ds, for acceleration (or backward braking).
inline double reachable_speed(
  const MotionLimits & m, double from, double ds, double k, double q,
  double cap, bool braking = false)
{
  const auto bounds = curve_limits(m, k, q);
  const double a = braking ? bounds.deceleration : bounds.acceleration;
  double squared = from * from + 2.0 * a * std::max(0.0, ds);
  if (m.enable_dynamic_angular_budget && ds > 0.0) {
    const double c = k / (2.0 * ds);
    if (c + q > 0.0) {
      squared =
        std::min(
        squared,
        (angular_budget(m) * m.dynamic_budget_utilization + c * from * from) / (c + q));
    }
  }
  return std::min({cap, bounds.speed, std::sqrt(std::max(0.0, squared))});
}

inline bool speed_edge_is_feasible(
  const MotionLimits & m, const SpeedSegment & edge, double a, double b)
{
  const double acceleration = (b * b - a * a) / (2.0 * edge.length);
  const auto limits = curve_limits(m, edge.curvature, edge.rate);
  const double eps = 1e-8;  // roundoff only; not a tunable physical allowance
  return std::max(a, b) <= std::min(edge.cap, limits.speed) + eps &&
         acceleration <= limits.acceleration + eps &&
         -acceleration <= limits.deceleration + eps &&
         (!m.enable_dynamic_angular_budget ||
         std::abs(acceleration) * edge.curvature + std::max(a * a, b * b) * edge.rate <=
         angular_budget(m) * m.dynamic_budget_utilization + eps);
}

inline SpeedEnvelope speed_envelope(
  const std::vector<SpeedSegment> & input, MotionLimits m, bool start_stopped = true)
{
  validate_motion_limits(m);
  SpeedEnvelope out;
  out.distance.push_back(0.0);
  out.speed.push_back(m.v_nominal);
  std::vector<SpeedSegment> edges;
  for (const auto & e : input) {
    if (!std::isfinite(e.length) || e.length <= 0.0 || !std::isfinite(e.cap) || e.cap <= 0.0) {
      throw std::invalid_argument("Invalid speed segment length/cap");
    }
    const double cap = std::min({m.v_nominal, e.cap, curve_limits(m, e.curvature, e.rate).speed});
    out.speed.back() = std::min(out.speed.back(), cap);
    for (int half = 0; half < 2; ++half) {
      edges.push_back({e.length / 2.0, e.curvature, e.rate, cap, half == 1 && e.stop_after});
      out.distance.push_back(out.distance.back() + e.length / 2.0);
      out.speed.push_back(half == 1 && e.stop_after ? 0.0 : cap);
    }
  }
  if (start_stopped) {out.speed.front() = 0.0;}
  bool converged = false;
  for (std::size_t pass = 0; pass < m.speed_envelope_rounds; ++pass) {
    const auto previous = out.speed;
    for (std::size_t i = edges.size(); i > 0; --i) {
      const auto & e = edges[i - 1];
      out.speed[i - 1] = std::min(
        out.speed[i - 1], reachable_speed(
          m, out.speed[i], e.length, e.curvature, e.rate, e.cap, true));
    }
    for (std::size_t i = 0; i < edges.size(); ++i) {
      const auto & e = edges[i];
      out.speed[i + 1] = std::min(
        out.speed[i + 1], reachable_speed(
          m, out.speed[i], e.length, e.curvature, e.rate, e.cap));
    }
    double change = 0.0;
    for (std::size_t i = 0; i < out.speed.size(); ++i) {
      change = std::max(change, std::abs(out.speed[i] - previous[i]));
    }
    if (change <= m.speed_envelope_tolerance) {converged = true; break;}
  }
  bool valid = converged;
  for (std::size_t i = 0; i < edges.size(); ++i) {
    valid = valid && speed_edge_is_feasible(m, edges[i], out.speed[i], out.speed[i + 1]);
  }
  if (!valid && m.enable_dynamic_angular_budget) {
    m.enable_dynamic_angular_budget = false;
    auto fallback = speed_envelope(input, m, start_stopped);
    fallback.conservative_fallback = true;
    return fallback;
  }
  if (!valid) {throw std::runtime_error("Conservative speed envelope failed validation");}
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const double sum = out.speed[i] + out.speed[i + 1];
    if (sum <= 0.0) {out.time = std::numeric_limits<double>::infinity(); break;}
    out.time += 2.0 * edges[i].length / sum;
  }
  return out;
}
}  // namespace plan
#endif  // PLAN__SPEED_ENVELOPE_HPP_

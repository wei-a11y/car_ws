// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PATH_TURN_FEATURES_HPP_
#define PATH_TURN_FEATURES_HPP_
#include <algorithm>
#include <cmath>
#include <vector>
#include "bezier_geometry.hpp"

namespace plan
{
struct TurnFeature
{
  double begin, end, peak_s, peak_curvature;
  int sign;
  std::size_t section = 0;
};
struct TurnFeatures
{
  std::vector<TurnFeature> turns;
  std::vector<double> zero_crossings;
};
inline TurnFeatures path_turn_features(
  const std::vector<PathPoint2D> & path, const PathProcessingConfig & config)
{
  TurnFeatures out;
  if (path.size() < 2) {return out;}
  std::vector<double> s(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  bool active = false;TurnFeature turn{};std::size_t section = 0;
  const auto flush = [&](double end) {
      if (active && end - turn.begin >= config.window_feature_min_length) {
        turn.end = end;out.turns.push_back(turn);
      }
      active = false;
    };
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double k = path[i].curvature;
    if (i && path[i].stop_required) {flush(s[i]);++section;}
    if (active && k * turn.sign < config.window_feature_exit_curvature) {flush(s[i]);}
    if (!active && std::abs(k) >= config.window_feature_enter_curvature &&
      !path[i].stop_required)
    {
      turn = {s[i], s[i], s[i], std::abs(k), k > 0 ? 1 : -1, section};active = true;
    }
    if (active && std::abs(k) > turn.peak_curvature) {
      turn.peak_curvature = std::abs(k);turn.peak_s = s[i];
    }
  }
  flush(s.back());
  for (std::size_t i = 1; i < out.turns.size(); ++i) {
    const auto & a = out.turns[i - 1];const auto & b = out.turns[i];
    if (a.sign == b.sign || a.section != b.section) {continue;}
    // Find the real signed crossing between persistent turns, never prescribe its position.
    for (std::size_t j = 1; j < path.size(); ++j) {
      if (s[j] < a.peak_s || s[j - 1] > b.peak_s || path[j].stop_required) {continue;}
      const double left = path[j - 1].curvature, right = path[j].curvature;
      if (left * a.sign > 0 && right * a.sign <= 0) {
        out.zero_crossings.push_back(s[j - 1] + (s[j] - s[j - 1]) * left / (left - right));
        break;
      }
    }
  }
  return out;
}

// Interval bounds guide extra objective samples. They are not a safety certificate:
// the independent full-curve certifier must still accept every candidate.
inline std::vector<double> feature_curve_samples(
  const bezier::Bezier & curve, const PathProcessingConfig & config)
{
  std::vector<double> out;
  const auto refine = [&](auto && self, double a, double b, std::size_t depth) -> void {
      const double middle = (a + b) / 2;
      const auto x = bezier::geometry(curve, a), y = bezier::geometry(curve, middle);
      const auto z = bezier::geometry(curve, b);
      const auto bound = bezier::fixed_bounds(bezier::interval(curve, a, b));
      const double peak = std::max(
        {std::abs(x.curvature), std::abs(y.curvature),
          std::abs(z.curvature), 1e-6});
      const double rate = std::max({std::abs(x.rate), std::abs(y.rate), std::abs(z.rate), 1e-6});
      if (depth < config.window_feature_sample_depth &&
        (bound.curvature > peak * (1 + config.window_feature_bound_tolerance) ||
        bound.rate > rate * (1 + config.window_feature_bound_tolerance)))
      {
        self(self, a, middle, depth + 1);self(self, middle, b, depth + 1);
      } else {
        out.push_back(a);out.push_back(middle);
      }
    };
  for (std::size_t j = 0; j < config.window_objective_samples; ++j) {
    refine(
      refine, static_cast<double>(j) / config.window_objective_samples,
      static_cast<double>(j + 1) / config.window_objective_samples, 0);
  }
  out.push_back(1.0);
  return out;
}
}  // namespace plan
#endif  // PATH_TURN_FEATURES_HPP_

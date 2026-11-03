// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "../src/path_experiments.hpp"
#include "../src/path_geometry_optimizer.hpp"
#include "../src/path_speed_coordination.hpp"
#include <cmath>
#include <limits>
#include "gtest/gtest.h"

namespace
{
plan::PathProcessingConfig config()
{
  plan::PathProcessingConfig c{false, .1, 200, 10000};
  c.motion.v_nominal = .5;c.motion.v_max = .6;
  c.motion.lateral_acceleration = .1;c.motion.angular_acceleration = 2.5;
  c.motion.linear_acceleration = c.motion.linear_deceleration = .15;
  c.enable_turn_speed_coordination = true;c.coordination_policy = "bounded";
  c.coordination_max_gap = 3.0;
  return c;
}
std::vector<plan::PathPoint2D> profile(double gap = .45)
{
  std::vector<plan::PathPoint2D> out;
  for (int i = 0; i <= 180; ++i) {
    const double x = i * .05;
    plan::PathPoint2D p{x, 1, 0, 0, 0, i == 180};
    p.curvature_bound = x >= 1.5 && x <= 1.8 ? 2.0 :
      x >= 1.8 + gap && x <= 2.1 + gap ? 2.1 : 0.0;
    p.speed_limit = .01;  // stale caps must be regenerated from the geometry
    out.push_back(p);
  }
  return out;
}
std::vector<plan::PathPoint2D> base_profile(const std::vector<plan::PathPoint2D> & input)
{
  auto c = config();c.enable_turn_speed_coordination = false;
  plan::PathProcessingStats stats;return plan::process_frozen_speed(input, c, stats);
}
void same_geometry(
  const std::vector<plan::PathPoint2D> & first, const std::vector<plan::PathPoint2D> & second)
{
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_DOUBLE_EQ(first[i].x, second[i].x);EXPECT_DOUBLE_EQ(first[i].y, second[i].y);
    EXPECT_DOUBLE_EQ(first[i].yaw, second[i].yaw);
    EXPECT_DOUBLE_EQ(first[i].tangent_yaw, second[i].tangent_yaw);
    EXPECT_DOUBLE_EQ(first[i].curvature, second[i].curvature);
    EXPECT_DOUBLE_EQ(first[i].curvature_bound, second[i].curvature_bound);
    EXPECT_DOUBLE_EQ(first[i].curvature_rate_bound, second[i].curvature_rate_bound);
    EXPECT_EQ(first[i].stop_required, second[i].stop_required);
    EXPECT_EQ(first[i].segment_type, second[i].segment_type);
  }
}
TEST(BoundedCoordination, IntegratesSquaredSpeedInPartialSegments)
{
  plan::SpeedEnvelope e;e.distance = {0, 1};e.speed = {0, 1};
  EXPECT_NEAR(plan::envelope_interval_time(e, 0, 1), 2, 1e-12);
  EXPECT_NEAR(plan::envelope_interval_time(e, 0, .25), 1, 1e-12);
  EXPECT_NEAR(plan::envelope_interval_time(e, .25, 1), 1, 1e-12);
  e.speed = {.3, .3};
  EXPECT_NEAR(plan::envelope_interval_time(e, .2, .5), 1, 1e-12);
}
TEST(BoundedCoordination, PreservesGeometryReachabilityAndTimeBudget)
{
  const auto c = config();const auto input = profile();const auto base = base_profile(input);
  plan::PathProcessingStats stats;
  const auto result = plan::process_frozen_speed(input, c, stats);
  same_geometry(input, result);
  const auto before = plan::speed_envelope(plan::path_speed_segments(base), c.motion);
  const auto after = plan::speed_envelope(plan::path_speed_segments(result), c.motion);
  EXPECT_LE(after.time, before.time * 1.02 + 1e-8);
  EXPECT_LE(
    plan::envelope_interval_time(after, 1.5, 2.6),
    plan::envelope_interval_time(before, 1.5, 2.6) * 1.02 + 1e-8);
  EXPECT_LE(stats.speed_events_after, stats.speed_events_before);
  EXPECT_LE(plan::envelope_variation(after), plan::envelope_variation(before) + 1e-8);
  EXPECT_LT(plan::envelope_variation(after), plan::envelope_variation(before)) <<
    stats.coordination_report;
  for (std::size_t i = 1; i < after.speed.size(); ++i) {
    const auto & p = result[(i - 1) / 2];
    const plan::SpeedSegment edge{after.distance[i] - after.distance[i - 1],
      p.curvature_bound, p.curvature_rate_bound, p.speed_limit, false};
    EXPECT_TRUE(plan::speed_edge_is_feasible(c.motion, edge, after.speed[i - 1], after.speed[i]));
  }
}
TEST(BoundedCoordination, RepeatedReplayResetsPreviouslyReducedCaps)
{
  const auto c = config();plan::PathProcessingStats a, b;
  const auto first = plan::process_frozen_speed(profile(), c, a);
  const auto second = plan::process_frozen_speed(first, c, b);
  same_geometry(first, second);
  for (std::size_t i = 0; i < first.size(); ++i) {
    EXPECT_DOUBLE_EQ(first[i].speed_limit, second[i].speed_limit);
  }
}
TEST(BoundedCoordination, ZeroTimeLossRetainsUncoordinatedCaps)
{
  auto c = config();c.coordination_max_time_loss = 0;
  const auto input = profile();const auto base = base_profile(input);
  plan::PathProcessingStats stats;const auto result = plan::process_frozen_speed(input, c, stats);
  for (std::size_t i = 0; i < base.size(); ++i) {
    EXPECT_DOUBLE_EQ(base[i].speed_limit, result[i].speed_limit);
  }
}
TEST(BoundedCoordination, DoesNotBridgeMandatoryStopOrProfitableLongStraight)
{
  auto c = config();auto input = profile();input[42].stop_required = true;
  auto base = base_profile(input);plan::PathProcessingStats stats;
  auto result = plan::process_frozen_speed(input, c, stats);
  same_geometry(input, result);
  for (std::size_t i = 0; i < base.size(); ++i) {
    EXPECT_DOUBLE_EQ(base[i].speed_limit, result[i].speed_limit);
  }
  input = profile(3.0);c.coordination_max_gap = 1.0;base = base_profile(input);
  result = plan::process_frozen_speed(input, c, stats);
  for (std::size_t i = 0; i < base.size(); ++i) {
    EXPECT_DOUBLE_EQ(base[i].speed_limit, result[i].speed_limit);
  }
}
TEST(BoundedCoordination, RejectsBadCertificatesAndInvalidPolicy)
{
  auto c = config();auto input = profile();plan::PathProcessingStats stats;
  input[3].curvature = 1;
  EXPECT_THROW(plan::process_frozen_speed(input, c, stats), std::invalid_argument);
  input = profile();input.back().stop_required = false;
  EXPECT_THROW(plan::process_frozen_speed(input, c, stats), std::invalid_argument);
  input = profile();input[2].tangent_yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(plan::process_frozen_speed(input, c, stats), std::invalid_argument);
  c.coordination_policy = "unknown";
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
  c = config();c.coordination_trim_fractions = {1.1};
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
}
TEST(BoundedCoordination, TimeFallbackKeepsTheValidatedGeometry)
{
  auto c = config();c.coordination_policy = "legacy";
  const auto input = profile();const auto base = base_profile(input);
  plan::PathProcessingStats stats;
  const auto result = plan::process_frozen_speed(input, c, stats, 0);
  same_geometry(input, result);
  EXPECT_NE(stats.coordination_report.find("\"fallback\":\"time_limit\""), std::string::npos);
  for (std::size_t i = 0; i < base.size(); ++i) {
    EXPECT_DOUBLE_EQ(base[i].speed_limit, result[i].speed_limit);
  }
}
}  // namespace

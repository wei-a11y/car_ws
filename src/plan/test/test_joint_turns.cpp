// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/path_processing.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "gtest/gtest.h"

namespace
{
plan::PlanningGrid free_grid()
{
  return plan::PlanningGrid::from_inflated_grid(
    {160, 160, .05, 0, 0, 0}, std::vector<std::int8_t>(25600, 0), 25600);
}

plan::PathProcessingConfig config()
{
  plan::PathProcessingConfig c{false, .1, 200, 100000};
  c.enable_arc_smoothing = c.enable_curvature_transition = c.enable_joint_optimization = true;
  c.arc_radius_max = .65;
  c.enable_local_relocation = false;
  return c;
}

void check_path(
  const plan::PlanningGrid & grid, const std::vector<plan::PathPoint2D> & p,
  const plan::PathProcessingConfig & c, const plan::PathProcessingStats & stats)
{
  ASSERT_GE(p.size(), 2U);
  EXPECT_TRUE(p.back().stop_required);
  EXPECT_DOUBLE_EQ(p.back().speed_limit, 0);
  EXPECT_LE(stats.selected_length, stats.baseline_length * c.max_length_ratio + 1e-9);
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    const auto & a = p[i]; const auto & b = p[i + 1];
    const double ds = std::hypot(b.x - a.x, b.y - a.y);
    ASSERT_GT(ds, 1e-10);
    EXPECT_TRUE(plan::line_of_sight(grid, {a.x, a.y}, {b.x, b.y}));
    EXPECT_LE(a.curvature_bound, 1 / c.arc_min_radius + 1e-8);
    EXPECT_GE(a.speed_limit, c.motion.min_turn_speed - 1e-9);
    const auto limit = plan::curve_limits(c.motion, a.curvature_bound, a.curvature_rate_bound);
    EXPECT_LE(a.speed_limit, limit.speed + 1e-9);
    EXPECT_LE(
      a.speed_limit * a.curvature_bound,
      c.motion.feedforward_angular_fraction * c.motion.w_max + 1e-9);
    EXPECT_LE(
      std::max(limit.acceleration, limit.deceleration) * a.curvature_bound +
      a.speed_limit * a.speed_limit * a.curvature_rate_bound,
      c.motion.angular_acceleration_fraction * c.motion.angular_acceleration + 1e-9);
    if (!b.stop_required) {
      EXPECT_LE(std::abs(b.curvature - a.curvature) / ds, a.curvature_rate_bound + 1e-7);
      EXPECT_LT(std::abs(std::remainder(b.tangent_yaw - a.tangent_yaw, 2 * std::acos(-1.))), .11);
    }
  }
}

TEST(JointTurns, SharesShortSegmentForBothSameDirectionAndSBends)
{
  const auto grid = free_grid(); auto c = config();
  for (bool same_direction : {false, true}) {
    const std::vector<plan::Point2D> raw{
      {.5, 1.5}, {3, 1.5}, {3, 1.95}, {same_direction ? .5 : 6., 1.95}};
    plan::PathProcessingStats old, now;
    c.enable_joint_optimization = false;
    plan::process_path(grid, raw, c, &old);
    c.enable_joint_optimization = true;
    const auto p = plan::process_path(grid, raw, c, &now);
    ASSERT_GT(old.retained_corners, 0U);
    EXPECT_EQ(now.retained_corners, 0U);
    EXPECT_EQ(now.arc_fallbacks, 0U);
    EXPECT_FALSE(now.budget_exhausted);
    check_path(grid, p, c, now);
  }
}

TEST(JointTurns, SmallAngleUsesGentleDoubleTransitionBeyondRadiusSearchLimit)
{
  const auto grid = free_grid(); auto c = config(); plan::PathProcessingStats stats;
  const auto p = plan::process_path(grid, {{.5, 1.5}, {3, 1.5}, {6, 1.6}}, c, &stats);
  EXPECT_EQ(stats.retained_corners, 0U);
  double peak = 0;
  for (const auto & q : p) {
    peak = std::max(peak, std::abs(q.curvature));
  }
  EXPECT_GT(peak, 0);
  EXPECT_LT(peak, 1 / c.arc_radius_max);
  EXPECT_NEAR(p.front().tangent_yaw, 0, 1e-10);
  EXPECT_NEAR(p.back().tangent_yaw, std::atan2(.1, 3.), 1e-10);
  check_path(grid, p, c, stats);
}

TEST(JointTurns, CurvatureRateIsComfortPreferenceButMinimumSpeedIsHardConstraint)
{
  const auto grid = free_grid(); auto c = config();
  c.max_curvature_rate = .001;
  c.enable_arc_fallback = false;
  plan::PathProcessingStats stats;
  auto p = plan::process_path(grid, {{.5, 1.5}, {3, 1.5}, {3, 4}}, c, &stats);
  EXPECT_EQ(stats.retained_corners, 0U);
  check_path(grid, p, c, stats);
  c.motion.angular_acceleration = .0001;
  p = plan::process_path(grid, {{.5, 1.5}, {3, 1.5}, {3, 4}}, c, &stats);
  EXPECT_EQ(stats.retained_corners, 1U);
  ASSERT_GE(stats.corners.size(), 2U);
  EXPECT_GT(stats.corners[1].speed, 0U);
}

TEST(JointTurns, ShapedCircleFallbackAndBudgetKeepSafeExecutableGeometry)
{
  const auto grid = free_grid(); auto c = config(); plan::PathProcessingStats stats;
  c.transition_min_length = c.transition_length = 2.;
  c.max_curvature_rate = .001;  // Legacy and continuous candidates cannot fit this short bend.
  const std::vector<plan::Point2D> raw{{2.5, 1.5}, {3, 1.5}, {3, 2}};
  auto p = plan::process_path(grid, raw, c, &stats);
  EXPECT_EQ(stats.retained_corners, 0U);
  EXPECT_EQ(stats.arc_fallbacks, 1U);
  check_path(grid, p, c, stats);
  // Reconstruct the full Hermite connectors independently in the power basis,
  // then densely subdivide them. This does not just check planner output points.
  const auto entry = std::find_if(
    p.begin(), p.end(), [](const auto & q) {
      return q.segment_type == 2;
    });
  const auto circle = std::find_if(
    entry, p.end(), [](const auto & q) {
      return q.segment_type == 2 && q.curvature_rate_bound == 0 && q.curvature > 0;
    });
  ASSERT_NE(circle, p.end());
  const double radius = 1 / circle->curvature;
  const double blend = 3 - radius - entry->x;
  const double pi = std::acos(-1.);
  const auto at = [&](double s) {
      return plan::Point2D{3 - radius + radius * std::sin(s / radius),
      1.5 + radius - radius * std::cos(s / radius)};
    };
  const auto dense = [&](plan::Point2D a, double ya, double ka,
      plan::Point2D b, double yb, double kb) {
      const double h = 2 * blend;
      plan::Point2D previous = a;
      for (int j = 0; j <= 2000; ++j) {
        const double t = j / 2000.;
        const auto coordinate = [&](double x0, double x1, double d0, double d1,
            double dd0, double dd1) {
            const double delta = x1 - x0;
            const double c3 = 10 * delta - 6 * d0 - 4 * d1 - 1.5 * dd0 + .5 * dd1;
            const double c4 = -15 * delta + 8 * d0 + 7 * d1 + 1.5 * dd0 - dd1;
            const double c5 = 6 * delta - 3 * d0 - 3 * d1 - .5 * dd0 + .5 * dd1;
            return x0 + t * (d0 + t * (dd0 / 2 + t * (c3 + t * (c4 + t * c5))));
          };
        const plan::Point2D q{
          coordinate(
            a.x, b.x, h * std::cos(ya), h * std::cos(yb),
            -h * h * ka * std::sin(ya), -h * h * kb * std::sin(yb)),
          coordinate(
            a.y, b.y, h * std::sin(ya), h * std::sin(yb),
            h * h * ka * std::cos(ya), h * h * kb * std::cos(yb))};
        const double dx = q.x - (3 - radius), dy = q.y - (1.5 + radius);
        double deviation = std::abs(std::hypot(dx, dy) - radius);
        if (q.x <= 3 - radius) {deviation = std::min(deviation, std::abs(q.y - 1.5));}
        if (q.y >= 1.5 + radius) {deviation = std::min(deviation, std::abs(q.x - 3));}
        EXPECT_LE(deviation, .01);
        EXPECT_TRUE(plan::line_of_sight(grid, previous, q));
        previous = q;
      }
    };
  dense({entry->x, entry->y}, 0, 0, at(blend), blend / radius, 1 / radius);
  dense(
    at(radius * pi / 2 - blend), pi / 2 - blend / radius, 1 / radius,
    {3, 1.5 + radius + blend}, pi / 2, 0);
  c.fallback_deviation = 1e-8;
  p = plan::process_path(grid, raw, c, &stats);
  EXPECT_EQ(stats.retained_corners, 1U);
  EXPECT_GT(stats.corners[1].deviation, 0U);
  c.fallback_deviation = .01;
  c.joint_work_budget = 1;
  p = plan::process_path(grid, raw, c, &stats);
  EXPECT_TRUE(stats.budget_exhausted);
  EXPECT_EQ(stats.retained_corners, 1U);
  EXPECT_LE(stats.work_used, c.joint_work_budget);
  check_path(grid, p, c, stats);
}

TEST(JointTurns, LocalRelocationPreservesEndpointsAndLengthBudget)
{
  const auto grid = free_grid(); auto c = config(); plan::PathProcessingStats stats;
  c.enable_local_relocation = true;
  const std::vector<plan::Point2D> raw{{.5, 1.5}, {3, 1.5}, {3, 1.9}, {6, 1.9}};
  const auto p = plan::process_path(grid, raw, c, &stats);
  EXPECT_GT(stats.variants_evaluated, 1U);
  EXPECT_DOUBLE_EQ(p.front().x, raw.front().x);
  EXPECT_DOUBLE_EQ(p.front().y, raw.front().y);
  EXPECT_DOUBLE_EQ(p.back().x, raw.back().x);
  EXPECT_DOUBLE_EQ(p.back().y, raw.back().y);
  EXPECT_LE(stats.work_used, c.joint_work_budget);
  check_path(grid, p, c, stats);
}

TEST(JointTurns, InflatedObstaclesRetainCornerAndReportCollision)
{
  auto cells = std::vector<std::int8_t>(25600, 100);
  // A zero-clearance L corridor: the original centre line is safe, cutting the bend is not.
  for (int x = 10; x <= 60; ++x) {
    cells[30 * 160 + x] = 0;
  }
  for (int y = 30; y <= 80; ++y) {
    cells[y * 160 + 60] = 0;
  }
  const auto grid = plan::PlanningGrid::from_inflated_grid({160, 160, .05, 0, 0, 0}, cells, 25600);
  auto c = config(); plan::PathProcessingStats stats;
  const auto p =
    plan::process_path(grid, {{.525, 1.525}, {3.025, 1.525}, {3.025, 4.025}}, c, &stats);
  EXPECT_EQ(stats.retained_corners, 1U);
  ASSERT_GE(stats.corners.size(), 2U);
  EXPECT_TRUE(stats.corners[1].retained);
  EXPECT_GT(stats.corners[1].collision, 0U);
  check_path(grid, p, c, stats);
}
}  // namespace

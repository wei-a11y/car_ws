// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/path_processing.hpp"
#include "plan/speed_envelope.hpp"
#include "../src/bezier_geometry.hpp"
#include "../src/path_experiments.hpp"
#include "../src/path_geometry_optimizer.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include "gtest/gtest.h"

namespace
{
plan::MotionLimits motion()
{
  plan::MotionLimits m;
  m.v_nominal = .5;m.v_max = .6;m.lateral_acceleration = .1;m.angular_acceleration = 2.5;
  m.linear_acceleration = m.linear_deceleration = .15;
  return m;
}
plan::PlanningGrid grid()
{
  return plan::PlanningGrid::from_inflated_grid(
    {160, 160, .05, 0, 0, 0}, std::vector<std::int8_t>(25600, 0), 25600);
}
plan::PathProcessingConfig config()
{
  plan::PathProcessingConfig c{false, .1, 200, 100000};
  c.enable_arc_smoothing = c.enable_curvature_transition = c.enable_joint_optimization = true;
  c.enable_local_relocation = false;c.motion = motion();c.arc_radius = .5;c.arc_radius_max = .7;
  return c;
}
TEST(DynamicBudget, SteadyCurveUsesFullBudgetWithoutChangingPhysicalLimits)
{
  auto m = motion();const double old = plan::curve_limits(m, 2, 50).speed;
  m.enable_dynamic_angular_budget = true;
  const double unlocked = plan::curve_limits(m, 2, 50).speed;
  EXPECT_NEAR(unlocked / old, std::sqrt(2.0 * m.dynamic_budget_utilization), 1e-10);
  EXPECT_LE(unlocked * unlocked * 50, plan::angular_budget(m) + 1e-10);
  const double next = plan::reachable_speed(m, .1, .05, 2, 50, .6);
  const plan::SpeedSegment edge{.05, 2, 50, .6, false};
  EXPECT_TRUE(plan::speed_edge_is_feasible(m, edge, .1, next));
}
TEST(DynamicBudget, EnvelopesCertifyEveryHalfSegmentAndStopInterior)
{
  auto m = motion();m.enable_dynamic_angular_budget = true;
  for (double k:{0., 1., 3.}) {
    for (double q:{0., 5., 30.}) {
      const std::vector<plan::SpeedSegment> segments{{.5, k, q, .5, true}, {.7, k, q, .5, true}};
      const auto e = plan::speed_envelope(segments, m);
      ASSERT_EQ(e.speed.size(), 5U);EXPECT_EQ(e.speed.front(), 0);EXPECT_EQ(e.speed[2], 0);
      EXPECT_GT(e.speed[1], 0);EXPECT_GT(e.speed[3], 0);EXPECT_TRUE(std::isfinite(e.time));
      auto effective = m;effective.enable_dynamic_angular_budget = !e.conservative_fallback;
      for (std::size_t i = 0; i + 1 < e.speed.size(); ++i) {
        const plan::SpeedSegment edge{e.distance[i + 1] - e.distance[i], k, q, .5, false};
        EXPECT_TRUE(plan::speed_edge_is_feasible(effective, edge, e.speed[i], e.speed[i + 1]));
      }
    }
  }
}
TEST(BezierGeometry, SharedKnotsPreservePositionTangentAndCurvature)
{
  const double a[]{1, 1, 0, .5, 0}, b[]{1.5, 1.2, .4, .5, .8}, c[]{2, 1.4, .2, .5, -.4};
  const auto first = plan::bezier::from_knots(a, b), second = plan::bezier::from_knots(b, c);
  const auto left = plan::bezier::geometry(first, 1), right = plan::bezier::geometry(second, 0);
  EXPECT_NEAR(left.point.x, right.point.x, 1e-12);EXPECT_NEAR(left.point.y, right.point.y, 1e-12);
  EXPECT_NEAR(left.yaw, right.yaw, 1e-12);EXPECT_NEAR(left.curvature, right.curvature, 1e-12);
  EXPECT_NEAR(left.curvature, .8, 1e-12);
}
TEST(WindowBezier, AllEightSwitchCombinationsRemainSafeAndRespectFrozenBudget)
{
  const auto map = grid();
  const std::vector<plan::Point2D> raw{{1, 1}, {3, 1}, {3, 2}, {5, 2}};
  for (unsigned flags = 0; flags < 8; ++flags) {
    auto c = config();
    c.enable_window_bezier = flags & 1;c.motion.enable_dynamic_angular_budget = flags & 2;
    c.enable_turn_speed_coordination = flags & 4;
    plan::PathProcessingStats stats;
    const auto p = plan::process_path(map, raw, c, &stats);
    ASSERT_GE(p.size(), 2U);EXPECT_EQ(p.front().x, 1);EXPECT_EQ(p.back().x, 5);
    EXPECT_TRUE(p.back().stop_required);EXPECT_EQ(p.back().speed_limit, 0);
    for (std::size_t i = 1; i < p.size(); ++i) {
      EXPECT_TRUE(plan::line_of_sight(map, {p[i - 1].x, p[i - 1].y}, {p[i].x, p[i].y}));
      EXPECT_GT(p[i - 1].speed_limit, 0);
    }
    std::cout << "flags=" << flags << " " << stats.window_report << std::endl;
    if (flags) {
      EXPECT_LE(stats.selected_length, 1.1 * stats.baseline_length + 1e-8);
      EXPECT_LE(stats.estimated_time, 1.1 * stats.frozen_baseline_time + 1e-8);
    }
  }
}
TEST(WindowBezier, BudgetExhaustionRetainsCertifiedBaseline)
{
  auto c = config();c.enable_window_bezier = true;c.window_check_budget = 1;
  const auto map = grid();const std::vector<plan::Point2D> raw{{1, 1}, {3, 1}, {3, 3}};
  plan::PathProcessingStats stats;const auto p = plan::process_path(map, raw, c, &stats);
  auto old = c;old.enable_window_bezier = false;
  const auto baseline = plan::process_path(map, raw, old);
  ASSERT_EQ(p.size(), baseline.size());
  for (std::size_t i = 0; i < p.size(); ++i) {
    EXPECT_EQ(p[i].x, baseline[i].x);EXPECT_EQ(p[i].y, baseline[i].y);
  }
  EXPECT_NE(stats.window_report.find("check_budget"), std::string::npos);
}
TEST(WindowBezier, InvalidTuningIsRejected)
{
  auto c = config();c.window_max_deviation = -1;
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
  c = config();c.window_seed_scales = {0};
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
  c = config();c.enable_joint_optimization = false;c.enable_curvature_transition = false;
  c.motion.enable_dynamic_angular_budget = true;
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
}
TEST(WindowBezier, SingletonWithoutJointOptimizerHasValidTerminalMetadata)
{
  const auto map = grid();
  for (unsigned flags = 1; flags < 8; ++flags) {
    auto c = config();c.enable_joint_optimization = false;
    c.enable_window_bezier = flags & 1;c.motion.enable_dynamic_angular_budget = flags & 2;
    c.enable_turn_speed_coordination = flags & 4;
    const auto path = plan::process_path(map, {{1.025, 1.025}}, c);
    ASSERT_EQ(path.size(), 1U);EXPECT_TRUE(path[0].stop_required);
    EXPECT_DOUBLE_EQ(path[0].speed_limit, 0.0);
  }
}
TEST(WindowBezier, NarrowLCorridorCannotBeRoundedThroughBlockedCells)
{
  auto cells = std::vector<std::int8_t>(25600, 100);
  for (int x = 20; x <= 60; ++x) {
    cells[20 * 160 + x] = 0;
  }
  for (int y = 20; y <= 70; ++y) {
    cells[y * 160 + 60] = 0;
  }
  const auto map = plan::PlanningGrid::from_inflated_grid({160, 160, .05, 0, 0, 0}, cells, 25600);
  auto c = config();c.enable_window_bezier = true;c.motion.enable_dynamic_angular_budget = true;
  plan::PathProcessingStats stats;
  const auto p =
    plan::process_path(map, {{1.025, 1.025}, {3.025, 1.025}, {3.025, 3.525}}, c, &stats);
  EXPECT_GE(stats.retained_corners, 1U);
  for (std::size_t i = 1; i < p.size(); ++i) {
    EXPECT_TRUE(plan::line_of_sight(map, {p[i - 1].x, p[i - 1].y}, {p[i].x, p[i].y}));
  }
}
TEST(GeometryFirst, RefinesCollisionEnvelopeButRejectsRealCurveCollision)
{
  const plan::bezier::Bezier curve{{{1, 1}, {1.4, 1.8}, {1.8, 2}, {2.2, 2}, {2.6, 1.8}, {3, 1}}};
  auto c = config();c.window_detailed_diagnostics = true;
  c.window_max_deviation = 10;c.resample_spacing = 10;c.arc_max_deviation = 1;
  c.arc_max_angle = 3;c.arc_min_radius = .05;c.max_curvature_rate = 1000;
  std::vector<plan::PathPoint2D> reference;
  for (double t:{0., .25, .5, .75, 1.}) {
    const auto g = plan::bezier::geometry(curve, t);
    reference.push_back({g.point.x, g.point.y, g.yaw, g.yaw, g.curvature, false});
  }
  auto cells = std::vector<std::int8_t>(25600, 0);cells[20 * 160 + 40] = 100;
  auto map = plan::PlanningGrid::from_inflated_grid({160, 160, .05, 0, 0, 0}, cells, 25600);
  const auto safe = plan::certify_geometry(map, {curve}, reference, c);
  EXPECT_TRUE(safe.safe) << safe.reason;EXPECT_GT(safe.used, 1U);
  for (std::size_t i = 1; i < safe.points.size(); ++i) {
    EXPECT_TRUE(
      plan::line_of_sight(
        map, {safe.points[i - 1].x, safe.points[i - 1].y},
        {safe.points[i].x, safe.points[i].y}));
  }
  c.window_check_budget = 1;
  const auto unknown = plan::certify_geometry(map, {curve}, reference, c);
  EXPECT_FALSE(unknown.safe);EXPECT_EQ(unknown.reason, "check_budget");
  c.window_check_budget = 200000;
  const auto midpoint = plan::bezier::geometry(curve, .5).point;
  std::size_t cx, cy;ASSERT_TRUE(map.world_to_cell(midpoint.x, midpoint.y, cx, cy));
  cells[cy * 160 + cx] = 100;
  map = plan::PlanningGrid::from_inflated_grid({160, 160, .05, 0, 0, 0}, cells, 25600);
  const auto collision = plan::certify_geometry(map, {curve}, reference, c);
  EXPECT_FALSE(collision.safe);EXPECT_EQ(collision.reason, "collision_witness");
  EXPECT_NE(collision.failure.find("blocking_cell"), std::string::npos);
}
TEST(GeometryFirst, UnequalSpanLengthsShareG2WithoutSharingParametricDerivativeLength)
{
  const double a[]{1, 1, 0, 1, 0}, b[]{1.04, 1.001, .05, 1, 2.5}, c[]{1.5, 1.1, .3, 1, .4};
  const auto first = plan::geometry_policy_curve(a, b), second = plan::geometry_policy_curve(b, c);
  const auto left = plan::bezier::geometry(first, 1), right = plan::bezier::geometry(second, 0);
  EXPECT_NEAR(left.yaw, right.yaw, 1e-10);EXPECT_NEAR(left.curvature, 2.5, 1e-8);
  EXPECT_NEAR(left.curvature, right.curvature, 1e-8);
  EXPECT_GT(plan::bezier::dist(second[0], second[1]), 5 * plan::bezier::dist(first[4], first[5]));
}
TEST(GeometryFirst, GeometryIsIndependentOfSpeedSwitchesWhenWorkCompletes)
{
  auto c = config();c.window_geometry_policy = "geometry_first";c.enable_window_bezier = true;
  c.window_iterations = 4;c.window_solver_seconds = 5;c.window_total_seconds = 20;
  c.window_seed_scales = {1.0};c.window_split_max_depth = 0;
  const auto map = grid();const std::vector<plan::Point2D> raw{{1, 1}, {3, 1}, {3, 2}, {5, 2}};
  auto legacy = c;legacy.enable_window_bezier = false;
  const auto baseline = plan::process_path(map, raw, legacy);
  plan::PathProcessingStats a, b;
  const auto first = plan::process_experiments(map, baseline, c, a);
  c.motion.enable_dynamic_angular_budget = c.enable_turn_speed_coordination = true;
  const auto second = plan::process_experiments(map, baseline, c, b);
  EXPECT_EQ(plan::geometry_fingerprint(first), plan::geometry_fingerprint(second));
  EXPECT_EQ(a.final_geometry_id, b.final_geometry_id);
  EXPECT_FALSE(a.geometry_reference_s.empty());
  EXPECT_NE(a.window_report.find("candidates"), std::string::npos);
  EXPECT_NE(a.window_report.find("search_end"), std::string::npos);
}
TEST(GeometryFirst, PreservesCandidateFailureHistoryAndFrozenBaselineOnCheckExhaustion)
{
  auto c = config();c.window_geometry_policy = "geometry_first";c.enable_window_bezier = true;
  c.window_check_budget = 1;
  const auto map = grid();const std::vector<plan::Point2D> raw{{1, 1}, {3, 1}, {3, 2}, {5, 2}};
  auto old = c;old.enable_window_bezier = false;
  const auto baseline = plan::process_path(map, raw, old);
  plan::PathProcessingStats stats;
  const auto result = plan::process_experiments(map, baseline, c, stats);
  EXPECT_EQ(plan::geometry_fingerprint(result), plan::geometry_fingerprint(baseline));
  EXPECT_NE(stats.window_report.find("check_budget"), std::string::npos);
  EXPECT_NE(stats.window_report.find("baseline_retained"), std::string::npos);
}
TEST(GeometryFirst, RejectsInvalidPolicyAndSubdivisionSettings)
{
  auto c = config();c.window_geometry_policy = "unknown";
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
  c = config();c.window_geometry_min_gain = 1;
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
  c = config();c.window_split_min_events = 1;
  EXPECT_THROW(plan::validate_path_processing_config(c), std::invalid_argument);
}
TEST(GeometryFirst, LargeWindowCanRetainRejectedParentAndAdoptCertifiedChildren)
{
  auto c = config();c.window_geometry_policy = "geometry_first";c.enable_window_bezier = true;
  c.window_max_knots = 10;c.window_solver_seconds = 2;c.window_total_seconds = 8;
  c.window_merge_gap = 3;
  c.window_seed_fit_tolerance = .01;
  const auto map = grid();
  const std::vector<plan::Point2D> raw{{1, 1}, {3, 1}, {3, 2}, {5, 2}, {5, 3}, {7, 3}};
  auto old = c;old.enable_window_bezier = false;
  const auto baseline = plan::process_path(map, raw, old);
  plan::PathProcessingStats stats;
  const auto result = plan::process_experiments(map, baseline, c, stats);
  EXPECT_NE(stats.window_report.find("knot_limit"), std::string::npos);
  EXPECT_NE(stats.window_report.find("\"parent\":0"), std::string::npos);
  EXPECT_NE(
    stats.window_report.find("\"child_adoption\":true"),
    std::string::npos) << stats.window_report;
  EXPECT_LE(stats.selected_length, stats.baseline_length * 1.1);
  for (std::size_t i = 1; i < result.size(); ++i) {
    EXPECT_TRUE(
      plan::line_of_sight(
        map, {result[i - 1].x, result[i - 1].y},
        {result[i].x, result[i].y}));
    EXPECT_GE(result[i - 1].curvature_bound + 1e-8, std::abs(result[i - 1].curvature));
    if (!result[i].stop_required) {
      EXPECT_GE(
        result[i - 1].curvature_rate_bound + 1e-8,
        std::abs(result[i].curvature - result[i - 1].curvature) /
        std::hypot(result[i].x - result[i - 1].x, result[i].y - result[i - 1].y));
    }
  }
}
}  // namespace

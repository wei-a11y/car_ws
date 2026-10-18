// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/lqr_tracker.hpp"
#include "plan/path_processing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace
{
controller::TrackerConfig curve_config()
{
  return {0.05, 0.15, 0.2, 0.6, 100.0, 25.0, 4.0, 1e-10, 5000,
    0.25, 0.5, 0.05, 1.0, 1.5, 0.05, 0.35, 0.03, 0.01, 0.02, 0.03, 100000};
}

std::vector<controller::ReferencePoint> planned_curve(double radius, bool right = false)
{
  const auto grid = plan::PlanningGrid::from_inflated_grid(
    {100, 100, 0.05, 0.0, 0.0, 0.0}, std::vector<std::int8_t>(10000, 0), 10000);
  plan::PathProcessingConfig cfg{false, 0.1, 200, 100000};
  cfg.enable_arc_smoothing = true;
  cfg.arc_radius = cfg.arc_min_radius = radius;
  const double y = right ? 3.5 : 1.5;
  const auto path = plan::process_path(
    grid,
    {{0.5, y}, {2.5, y}, {2.5, right ? 1.5 : 3.5}}, cfg);
  std::vector<controller::ReferencePoint> result;
  for (const auto & p : path) {
    result.push_back({p.x, p.y, p.tangent_yaw, p.curvature, p.stop_required});
  }
  return result;
}

TEST(ContinuousTracking, PlannerArcsPassWithoutIntermediateStopsInNonlinearSimulation)
{
  for (double radius : {0.20, 0.35, 0.60}) {
    for (bool right : {false, true}) {
      const auto cfg = curve_config();
      const auto path = planned_curve(radius, right);
      controller::LqrTracker tracker(cfg);
      tracker.set_profile(path);
      controller::Pose2D pose{path.front().x, path.front().y + 0.01, 0.0};
      controller::Velocity2D actual{0.0, 0.0};
      bool reached = false;
      double maximum_error = 0.0, cruise = 0.0, minimum_curve_speed = cfg.v_nominal;
      for (int tick = 0; tick < 5000; ++tick) {
        const auto r = tracker.step(pose, actual);
        ASSERT_NE(r.state, controller::TrackerState::TRACKING_ERROR)
          << "tick=" << tick << " radius=" << radius << " right=" << right
          << " fault=" << controller::fault_name(r.diagnostics.fault);
        maximum_error = std::max(maximum_error, r.diagnostics.nearest_distance);
        if (r.remaining > cfg.goal_position_tolerance + 0.01) {
          ASSERT_EQ(r.state, controller::TrackerState::TRACKING) << "tick=" << tick;
          EXPECT_GT(r.command.v, 0.0);
        }
        if (std::abs(r.reference_curvature) > 0.1) {
          minimum_curve_speed = std::min(minimum_curve_speed, r.command.v);
          EXPECT_LE(
            std::abs(r.feedforward_angular),
            cfg.feedforward_angular_fraction * cfg.w_max + 1e-12);
          EXPECT_EQ(r.command.w > 0.0, !right);
        } else if (r.progress > 0.5 && r.progress < 1.0) {
          cruise = std::max(cruise, r.command.v);
        }
        if (r.state == controller::TrackerState::TRACKING) {
          EXPECT_LE(r.command.v - actual.v, cfg.linear_acceleration * cfg.control_period + 1e-12);
          EXPECT_LE(actual.v - r.command.v, cfg.linear_deceleration * cfg.control_period + 0.001);
          EXPECT_LE(
            std::abs(r.command.w - actual.w),
            cfg.angular_acceleration * cfg.control_period + 1e-12);
        }
        actual = r.command;
        pose.x += actual.v * std::cos(pose.yaw) * cfg.control_period;
        pose.y += actual.v * std::sin(pose.yaw) * cfg.control_period;
        pose.yaw = controller::normalize_angle(pose.yaw + actual.w * cfg.control_period);
        if (r.state == controller::TrackerState::GOAL_REACHED) {
          reached = true;
          break;
        }
      }
      EXPECT_TRUE(reached);
      EXPECT_NEAR(cruise, cfg.v_nominal, 1e-12);
      EXPECT_LE(maximum_error, cfg.max_tracking_error);
      if (radius == 0.20) {
        EXPECT_LT(minimum_curve_speed, cfg.v_nominal);
      }
    }
  }
}

TEST(ContinuousTracking, BrakesBeforeTightCurveAndUsesCurvatureFeedforward)
{
  auto cfg = curve_config();
  cfg.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(cfg);
  tracker.set_profile(planned_curve(0.20));
  tracker.step({0.5, 1.5, 0.0}, {0.0, 0.0});  // Confirm initial alignment before testing motion.
  const auto cruise = tracker.step({1.0, 1.5, 0.0}, {0.15, 0.0});
  EXPECT_NEAR(cruise.command.v, 0.15, 1e-12);
  const auto approach = tracker.step({2.28, 1.5, 0.0}, {0.15, 0.0});
  EXPECT_LT(approach.reference_speed, cfg.v_nominal);
  const double theta = 0.7;
  const auto turn = tracker.step(
    {2.3 + 0.2 * std::sin(theta), 1.7 - 0.2 * std::cos(theta), theta}, {0.08, 0.4});
  ASSERT_EQ(turn.state, controller::TrackerState::TRACKING);
  EXPECT_NEAR(turn.reference_curvature, 5.0, 1e-12);
  EXPECT_NEAR(turn.feedforward_angular, turn.command.v * 5.0, 1e-12);
  EXPECT_LE(turn.reference_speed, 0.09 + 1e-12);
}

TEST(ContinuousTracking, RetainedCornerCannotBeSkippedAndRequiresMeasuredStop)
{
  auto cfg = curve_config();
  cfg.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(cfg);
  tracker.set_profile(
    {{0.0, 0.0, 0.0, 0.0, false},
      {1.0, 0.0, std::acos(-1.0) / 2.0, 0.0, true},
      {1.0, 1.0, std::acos(-1.0) / 2.0, 0.0, true}});
  const auto before = tracker.step({0.9, 0.0, 0.0}, {0.10, 0.0});
  EXPECT_NEAR(before.remaining, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(before.lookahead.y, 0.0);
  EXPECT_EQ(tracker.step({1.0, 0.0, 0.0}, {0.1, 0.0}).state, controller::TrackerState::BRAKING);
  EXPECT_EQ(tracker.step({1.0, 0.0, 0.0}, {0.0, 0.0}).state, controller::TrackerState::ALIGNING);
  const auto align = tracker.step({1.0, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_DOUBLE_EQ(align.command.v, 0.0);
  EXPECT_GT(align.command.w, 0.0);
  const auto leave = tracker.step({1.0, 0.0, std::acos(-1.0) / 2.0}, {0.0, 0.0});
  EXPECT_EQ(leave.state, controller::TrackerState::TRACKING);
  EXPECT_GT(leave.command.v, 0.0);
}

TEST(ContinuousTracking, ProjectionWindowPreventsJumpingToDistantOrUnvisitedPath)
{
  controller::LqrTracker tracker(curve_config());
  tracker.set_profile({{0.0, 0.0, 0.0, 0.0, false}, {2.0, 0.0, 0.0, 0.0, true}});
  EXPECT_EQ(
    tracker.step({1.0, 0.0, 0.0}, {0.0, 0.0}).state,
    controller::TrackerState::TRACKING_ERROR);
}

TEST(ContinuousTracking, InvalidProfilesClearPreviousReference)
{
  controller::LqrTracker tracker(curve_config());
  tracker.set_profile(planned_curve(0.35));
  EXPECT_THROW(tracker.set_profile({{0.0, 0.0, 0.0, 0.0, false}}), std::invalid_argument);
  EXPECT_FALSE(tracker.has_path());
  EXPECT_THROW(
    tracker.set_profile(
      {{0.0, 0.0, 0.0, NAN, false},
        {1.0, 0.0, 0.0, 0.0, true}}), std::invalid_argument);
  EXPECT_THROW(
    tracker.set_profile(
      {{0.0, 0.0, 1.0, 0.0, false},
        {1.0, 0.0, 0.0, 0.0, true}}), std::invalid_argument);
}
}  // namespace

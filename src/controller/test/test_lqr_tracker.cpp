// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/lqr_tracker.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace
{
controller::TrackerConfig config()
{
  return {0.05, 0.15, 0.2, 0.6, 100.0, 25.0, 4.0, 1e-10, 5000,
    0.25, 0.5, 0.05, 1.0, 1.5, 0.05, 0.35, 0.03, 0.01, 0.02, 0.03, 100000};
}

void expect_zero(const controller::TrackerResult & result)
{
  EXPECT_DOUBLE_EQ(result.command.v, 0.0);
  EXPECT_DOUBLE_EQ(result.command.w, 0.0);
}

TEST(LqrTracker, ExactZohAndAngleNormalization)
{
  const auto model = controller::discrete_model(0.15, 0.05);
  EXPECT_DOUBLE_EQ(model.a[0], 1.0);
  EXPECT_NEAR(model.a[1], 0.0075, 1e-15);
  EXPECT_DOUBLE_EQ(model.a[2], 0.0);
  EXPECT_DOUBLE_EQ(model.a[3], 1.0);
  EXPECT_NEAR(model.b[0], 0.0001875, 1e-15);
  EXPECT_DOUBLE_EQ(model.b[1], 0.05);
  const double pi = std::acos(-1.0);
  EXPECT_DOUBLE_EQ(controller::normalize_angle(pi), -pi);
  EXPECT_NEAR(controller::normalize_angle(2.0 * pi + 0.1), 0.1, 1e-14);
  EXPECT_NEAR(controller::normalize_angle(-2.0 * pi - 0.1), -0.1, 1e-14);
  EXPECT_THROW(controller::discrete_model(-1.0, 0.05), std::invalid_argument);
}

TEST(LqrTracker, RiccatiMatchesPhase5AndIsStableAcrossPositiveLowSpeeds)
{
  const auto c = config();
  const auto gain = controller::lqr_gain(c);
  EXPECT_NEAR(gain[0], 4.66404099, 1e-5);
  EXPECT_NEAR(gain[1], 2.61486747, 1e-5);
  for (double v : {0.0001, 0.01, 0.05, 0.15}) {
    const double t = c.control_period;
    const double trace = 2.0 - t * gain[1] - 0.5 * v * t * t * gain[0];
    const double determinant = 1.0 - t * gain[1] + 0.5 * v * t * t * gain[0];
    EXPECT_GT(1.0 - trace + determinant, 0.0);
    EXPECT_GT(1.0 + trace + determinant, 0.0);
    EXPECT_GT(1.0 - determinant, 0.0);
  }
  auto changed = c;
  changed.r_angular *= 4.0;
  EXPECT_LT(controller::lqr_gain(changed)[0], gain[0]);
}

TEST(LqrTracker, RejectsInvalidConfigAndSolverFailure)
{
  auto c = config();
  c.v_nominal = 0.0;
  EXPECT_THROW(controller::LqrTracker{c}, std::invalid_argument);
  c = config(); c.q_lateral = -1.0;
  EXPECT_THROW(controller::LqrTracker{c}, std::invalid_argument);
  c = config(); c.r_angular = NAN;
  EXPECT_THROW(controller::LqrTracker{c}, std::invalid_argument);
  c = config(); c.heading_tolerance = c.align_trigger;
  EXPECT_THROW(controller::LqrTracker{c}, std::invalid_argument);
  c = config(); c.riccati_max_iterations = 1;
  EXPECT_THROW(controller::LqrTracker{c}, std::runtime_error);
}

TEST(LqrTracker, NoPathAndInvalidReplacementAlwaysClearOldCommands)
{
  controller::LqrTracker tracker(config());
  expect_zero(tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0}));
  tracker.set_path({{0.0, 0.0}, {2.0, 0.0}});
  EXPECT_GT(tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0}).command.v, 0.0);
  EXPECT_THROW(tracker.set_path({{0.0, 0.0}, {NAN, 0.0}}), std::invalid_argument);
  EXPECT_FALSE(tracker.has_path());
  expect_zero(tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0}));
  tracker.set_path({});
  EXPECT_FALSE(tracker.has_path());
}

TEST(LqrTracker, LateralAndHeadingFeedbackHaveCorrectSigns)
{
  controller::LqrTracker tracker(config());
  tracker.set_path({{0.0, 0.0}, {2.0, 0.0}});
  const auto left = tracker.step({0.0, 0.02, 0.0}, {0.0, 0.0});
  EXPECT_NEAR(left.lateral_error, 0.02, 1e-15);
  EXPECT_LT(left.command.w, 0.0);
  const auto right = tracker.step({0.0, -0.02, 0.0}, {0.0, 0.0});
  EXPECT_GT(right.command.w, 0.0);
  const auto heading = tracker.step({0.0, 0.0, 0.1}, {0.0, 0.0});
  EXPECT_NEAR(heading.heading_error, 0.1, 1e-15);
  EXPECT_LT(heading.command.w, 0.0);
}

TEST(LqrTracker, HeadingWrapAcrossPiAndRotationSaturation)
{
  const double pi = std::acos(-1.0);
  controller::LqrTracker tracker(config());
  tracker.set_path({{0.0, 0.0}, {-2.0, 0.0}});
  const auto result = tracker.step({0.0, 0.0, -pi + 0.02}, {0.0, 0.0});
  EXPECT_NEAR(result.heading_error, 0.02, 1e-14);
  EXPECT_GT(result.command.v, 0.0);
  tracker.set_path({{0.0, 0.0}, {2.0, 0.0}});
  const auto aligning = tracker.step({0.0, 0.0, pi / 2.0}, {0.0, 0.0});
  EXPECT_EQ(aligning.state, controller::TrackerState::ALIGNING);
  EXPECT_DOUBLE_EQ(aligning.command.v, 0.0);
  EXPECT_DOUBLE_EQ(aligning.command.w, -config().w_max);
}

TEST(LqrTracker, BoundedNearestAndMetricLookaheadDoNotSkipCorners)
{
  auto c = config();
  c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {0.03, 0.0}, {0.4, 0.0}, {0.4, 1.0}});
  const auto result = tracker.step({0.2, 0.01, 0.0}, {0.0, 0.0});
  EXPECT_NEAR(result.nearest.x, 0.2, 1e-14);
  EXPECT_DOUBLE_EQ(result.nearest.y, 0.0);
  EXPECT_NEAR(result.lookahead.x, 0.4, 1e-14);
  EXPECT_DOUBLE_EQ(result.lookahead.y, 0.0);
  EXPECT_NEAR(result.remaining, 0.2, 1e-14);
  controller::LqrTracker bounded(config());
  bounded.set_path({{0.0, 0.0}, {2.0, 0.0}});
  EXPECT_EQ(
    bounded.step({1.0, 0.0, 0.0}, {0.0, 0.0}).state,
    controller::TrackerState::TRACKING_ERROR);
}

TEST(LqrTracker, SlowsAtEndAndRequiresMeasuredStopThenLatchesZero)
{
  auto c = config(); c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}});
  const auto slow = tracker.step({0.9, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_NEAR(slow.command.v, 0.1, 1e-14);
  EXPECT_LT(slow.command.v, c.v_nominal);
  const auto braking = tracker.step({0.98, 0.0, 0.0}, {0.04, 0.0});
  EXPECT_EQ(braking.state, controller::TrackerState::BRAKING);
  expect_zero(braking);
  const auto stopped = tracker.step({0.98, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(stopped.state, controller::TrackerState::GOAL_REACHED);
  expect_zero(stopped);
  for (int i = 0; i < 20; ++i) {
    expect_zero(tracker.step({0.98, 0.0, 0.0}, {0.0, 0.0}));
  }
  tracker.set_path({{0.98, 0.0}, {2.0, 0.0}});
  EXPECT_GT(tracker.step({0.98, 0.0, 0.0}, {0.0, 0.0}).command.v, 0.0);
}

TEST(LqrTracker, EachBendBrakesAndAlignsBeforeAdvancing)
{
  auto c = config(); c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  EXPECT_GT(tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0}).command.v, 0.0);
  const auto braking = tracker.step({1.0, 0.0, 0.0}, {0.1, 0.0});
  expect_zero(braking);
  EXPECT_EQ(braking.state, controller::TrackerState::BRAKING);
  expect_zero(tracker.step({1.0, 0.0, 0.0}, {0.0, 0.0}));
  const auto turn = tracker.step({1.0, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_DOUBLE_EQ(turn.command.v, 0.0);
  EXPECT_GT(turn.command.w, 0.0);
  const double yaw = std::acos(-1.0) / 2.0;
  expect_zero(tracker.step({1.0, 0.0, yaw}, {0.0, 0.2}));
  EXPECT_GT(tracker.step({1.0, 0.0, yaw}, {0.0, 0.0}).command.v, 0.0);
}

TEST(LqrTracker, LargeDeviationOvershootAndNonfiniteInputLatchZero)
{
  auto c = config(); c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}});
  const auto outside = tracker.step({0.0, 0.2, 0.0}, {0.0, 0.0});
  EXPECT_EQ(outside.state, controller::TrackerState::TRACKING_ERROR);
  expect_zero(outside);
  expect_zero(tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0}));
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}});
  const auto overshot = tracker.step({1.1, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(overshot.state, controller::TrackerState::TRACKING_ERROR);
  expect_zero(overshot);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}});
  expect_zero(tracker.step({0.0, 0.0, NAN}, {0.0, 0.0}));
}

TEST(LqrTracker, SinglePointIsPositionOnlyAndNeverInventsAHeading)
{
  controller::LqrTracker tracker(config());
  tracker.set_path({{0.0, 0.0}, {0.0, 0.0}});
  const auto reached = tracker.step({0.0, 0.0, 1.5}, {0.0, 0.0});
  EXPECT_EQ(reached.state, controller::TrackerState::GOAL_REACHED);
  expect_zero(reached);
  tracker.set_path({{0.5, 0.0}});
  const auto invalid = tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(invalid.state, controller::TrackerState::TRACKING_ERROR);
  expect_zero(invalid);
}

TEST(LqrTracker, CornerFaultRetainsFirstGeometrySnapshotUntilNewPath)
{
  auto c = config(); c.max_tracking_error = 0.04; c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(
    tracker.step({1.0, 0.0, 0.0}, {0.0, 0.0}).state,
    controller::TrackerState::ALIGNING);
  const auto fault = tracker.step({1.045, 0.0, 0.3}, {0.0, 0.1});
  EXPECT_EQ(fault.state, controller::TrackerState::TRACKING_ERROR);
  expect_zero(fault);
  const auto & d = fault.diagnostics;
  EXPECT_EQ(d.fault, controller::TrackerFault::NEAREST_DISTANCE_EXCEEDED);
  EXPECT_EQ(d.segment_index, 1U);
  EXPECT_EQ(d.segment_count, 2U);
  EXPECT_TRUE(d.aligning);
  EXPECT_TRUE(d.geometry_valid);
  EXPECT_NEAR(d.nearest_distance, 0.045, 1e-12);
  EXPECT_DOUBLE_EQ(d.start.x, 1.0);
  EXPECT_DOUBLE_EQ(d.end.y, 1.0);
  EXPECT_DOUBLE_EQ(d.actual.w, 0.1);
  const auto latched = tracker.step({1.0, 0.0, 1.57}, {0.0, 0.0});
  expect_zero(latched);
  EXPECT_DOUBLE_EQ(latched.diagnostics.pose.x, d.pose.x);
  EXPECT_DOUBLE_EQ(latched.diagnostics.actual.w, d.actual.w);
  const auto bad_input_after_fault = tracker.step({NAN, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(bad_input_after_fault.diagnostics.fault, d.fault);
  tracker.set_path({{1.0, 0.0}, {1.0, 1.0}});
  const auto recovered = tracker.step({1.0, 0.0, std::acos(-1.0) / 2.0}, {0.0, 0.0});
  EXPECT_EQ(recovered.diagnostics.fault, controller::TrackerFault::NONE);
  EXPECT_GT(recovered.command.v, 0.0);
}

TEST(LqrTracker, OvershootDiagnosticIsDistinctFromNearestDistanceGuard)
{
  auto c = config(); c.max_tracking_error = 0.2; c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}});
  const auto result = tracker.step({1.04, 0.01, 0.0}, {0.0, 0.0});
  expect_zero(result);
  EXPECT_EQ(result.diagnostics.fault, controller::TrackerFault::SEGMENT_END_OVERSHOOT);
  EXPECT_DOUBLE_EQ(result.diagnostics.projected, 1.04);
  EXPECT_DOUBLE_EQ(result.diagnostics.length, 1.0);
  EXPECT_GT(result.diagnostics.endpoint_distance, c.goal_position_tolerance);
  EXPECT_LT(result.diagnostics.nearest_distance, c.max_tracking_error);
}

TEST(LqrTracker, SinglePointAndNonfiniteDiagnosticsIdentifyFailedGuard)
{
  controller::LqrTracker tracker(config());
  tracker.set_path({{0.5, 0.0}});
  const auto point = tracker.step({0.0, 0.0, 0.0}, {0.0, 0.0});
  EXPECT_EQ(point.diagnostics.fault, controller::TrackerFault::SINGLE_POINT_NOT_REACHED);
  EXPECT_DOUBLE_EQ(point.diagnostics.endpoint_distance, 0.5);
  EXPECT_EQ(point.diagnostics.segment_count, 0U);
  tracker.set_path({{0.0, 0.0}, {1.0, 0.0}});
  const auto invalid = tracker.step({0.0, 0.0, NAN}, {0.0, 0.0});
  EXPECT_EQ(invalid.diagnostics.fault, controller::TrackerFault::NONFINITE_INPUT);
  EXPECT_FALSE(invalid.diagnostics.geometry_valid);
  EXPECT_TRUE(std::isnan(invalid.diagnostics.pose.yaw));
  expect_zero(invalid);
  EXPECT_STREQ(controller::fault_name(invalid.diagnostics.fault), "NONFINITE_INPUT");
}

TEST(LqrTracker, TrackingCommandsRespectConfiguredLimits)
{
  auto c = config(); c.w_max = 0.05; c.max_tracking_error = 0.2;
  controller::LqrTracker tracker(c);
  tracker.set_path({{0.0, 0.0}, {2.0, 0.0}});
  const auto result = tracker.step({0.0, 0.1, 0.0}, {0.0, 0.0});
  EXPECT_DOUBLE_EQ(result.command.w, -0.05);
  EXPECT_GE(result.command.v, 0.0);
  EXPECT_LE(result.command.v, c.v_max);
}

TEST(LqrTracker, ResamplingRoundoffAtLargeOriginDoesNotCreateFakeBends)
{
  auto c = config(); c.nearest_forward_distance = 10.0;
  controller::LqrTracker tracker(c);
  std::vector<controller::Point2D> points;
  for (int i = 0; i <= 100; ++i) {
    points.push_back({100.0 + i * 0.033, 150.0 + i * 0.017});
  }
  tracker.set_path(points);
  const auto result = tracker.step({100.0, 150.0, std::atan2(0.017, 0.033)}, {0.0, 0.0});
  EXPECT_NEAR(result.remaining, std::hypot(3.3, 1.7), 1e-10);
}

TEST(LqrTracker, IdealNonlinearUnicycleConvergesAndStopsOnStraightAndCornerPaths)
{
  for (const auto & path : std::vector<std::vector<controller::Point2D>>{
      {{0.0, 0.0}, {3.0, 0.0}}, {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}})
  {
    const auto c = config();
    controller::LqrTracker tracker(c);
    tracker.set_path(path);
    controller::Pose2D pose{0.0, 0.02, 0.0};
    controller::Velocity2D velocity{0.0, 0.0};
    bool reached = false;
    for (int i = 0; i < 6000; ++i) {
      const auto result = tracker.step(pose, velocity);
      ASSERT_NE(result.state, controller::TrackerState::TRACKING_ERROR) << "step " << i;
      velocity = result.command;
      pose.x += velocity.v * std::cos(pose.yaw) * c.control_period;
      pose.y += velocity.v * std::sin(pose.yaw) * c.control_period;
      pose.yaw = controller::normalize_angle(pose.yaw + velocity.w * c.control_period);
      if (result.state == controller::TrackerState::GOAL_REACHED) {
        reached = true;
        expect_zero(result);
        break;
      }
    }
    EXPECT_TRUE(reached);
    EXPECT_LE(
      std::hypot(pose.x - path.back().x, pose.y - path.back().y),
      c.goal_position_tolerance);
  }
}
}  // namespace

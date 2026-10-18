// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/path_processing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "plan/astar.hpp"

namespace
{
plan::PlanningGrid make_grid(std::vector<std::int8_t> cells = std::vector<std::int8_t>(64, 0))
{
  return plan::PlanningGrid::from_inflated_grid(
    {8, 8, 1.0, 0.0, 0.0, 0.0}, std::move(cells), 1000);
}

plan::PathProcessingConfig config(bool shortcut = false, double spacing = 100.0)
{
  return {shortcut, spacing, 200, 1000};
}

bool contains(const std::vector<plan::PathPoint2D> & path, double x, double y)
{
  return std::any_of(
    path.begin(), path.end(), [x, y](const auto & point) {
      return std::abs(point.x - x) < 1e-12 && std::abs(point.y - y) < 1e-12;
    });
}

TEST(PathProcessing, RemovesDuplicatesAndForwardCollinearPointsButNotReversals)
{
  const auto grid = make_grid();
  const auto path = plan::process_path(
    grid, {{0.5, 0.5}, {0.5, 0.5}, {1.5, 0.5}, {2.5, 0.5}, {2.5, 0.5}}, config());
  ASSERT_EQ(path.size(), 2U);
  EXPECT_DOUBLE_EQ(path.front().x, 0.5);
  EXPECT_DOUBLE_EQ(path.back().x, 2.5);
  const auto reversed = plan::process_path(grid, {{0.5, 0.5}, {2.5, 0.5}, {0.5, 0.5}}, config());
  ASSERT_EQ(reversed.size(), 3U);
  EXPECT_DOUBLE_EQ(reversed.front().yaw, 0.0);
  EXPECT_DOUBLE_EQ(reversed.back().yaw, std::acos(-1.0));
}

TEST(PathProcessing, HandlesEmptySingletonAndShortPath)
{
  const auto grid = make_grid();
  EXPECT_TRUE(plan::process_path(grid, {}, config()).empty());
  const auto point = plan::process_path(grid, {{0.5, 0.5}, {0.5, 0.5}}, config());
  ASSERT_EQ(point.size(), 1U);
  EXPECT_DOUBLE_EQ(point[0].yaw, 0.0);
  EXPECT_EQ(plan::process_path(grid, {{0.5, 0.5}, {0.6, 0.5}}, config(false, 0.2)).size(), 2U);
}

TEST(PathProcessing, ShortcutIsConfigurableAndLookaheadIsBounded)
{
  const auto grid = make_grid();
  const std::vector<plan::Point2D> raw{{0.5, 0.5}, {0.5, 4.5}, {4.5, 4.5}};
  const auto disabled = plan::process_path(grid, raw, config(false));
  const auto enabled = plan::process_path(grid, raw, config(true));
  EXPECT_EQ(disabled.size(), 3U);
  ASSERT_EQ(enabled.size(), 2U);
  EXPECT_NEAR(enabled.front().yaw, std::acos(-1.0) / 4.0, 1e-12);
  auto limited = config(true);
  limited.shortcut_max_lookahead = 1;
  EXPECT_EQ(plan::process_path(grid, raw, limited).size(), 3U);
}

TEST(PathProcessing, ShortcutChecksInflatedGridAndPreservesUnknownPolicy)
{
  std::vector<std::int8_t> cells(64, 0);
  cells[2 * 8 + 2] = 100;
  const auto blocked = make_grid(cells);
  EXPECT_FALSE(plan::line_of_sight(blocked, {0.5, 0.5}, {4.5, 4.5}));
  EXPECT_EQ(
    plan::process_path(
      blocked, {{0.5, 0.5}, {0.5, 4.5}, {4.5, 4.5}}, config(true)).size(), 3U);
  cells[2 * 8 + 2] = -1;
  const auto allowed = make_grid(cells);
  EXPECT_TRUE(plan::line_of_sight(allowed, {0.5, 0.5}, {4.5, 4.5}));
}

TEST(PathProcessing, SupercoverRejectsThinObstaclesCornerTouchesAndGridLineEdges)
{
  std::vector<std::int8_t> cells(64, 0);
  cells[0 * 8 + 1] = 100;
  const auto grid = make_grid(cells);
  EXPECT_FALSE(plan::line_of_sight(grid, {0.5, 0.5}, {1.5, 1.5}));  // Corner touch only.
  EXPECT_FALSE(plan::line_of_sight(grid, {1.5, 1.5}, {0.5, 0.5}));
  EXPECT_FALSE(plan::line_of_sight(grid, {0.5, 1.0}, {2.5, 1.0}));  // Along blocked edge.
  EXPECT_FALSE(plan::line_of_sight(grid, {2.5, 1.0}, {0.5, 1.0}));
  EXPECT_FALSE(plan::line_of_sight(grid, {0.1, 0.25}, {7.9, 0.3}));  // Cannot skip cell 1.
  EXPECT_TRUE(plan::line_of_sight(grid, {0.5, 1.5}, {7.5, 1.5}));
  EXPECT_TRUE(plan::line_of_sight(grid, {0.5, 2.0}, {7.5, 2.0}));
  EXPECT_FALSE(plan::line_of_sight(grid, {0.0, 2.0}, {7.5, 2.0}));  // Outside contact.
  EXPECT_FALSE(plan::line_of_sight(grid, {0.5, 2.0}, {8.0, 2.0}));
  EXPECT_FALSE(plan::line_of_sight(grid, {-1.0, 2.0}, {0.5, 2.0}));
  EXPECT_FALSE(plan::line_of_sight(grid, {NAN, 2.0}, {0.5, 2.0}));
}

TEST(PathProcessing, ResamplesUniformArcLengthAndKeepsCornersWithoutChords)
{
  std::vector<std::int8_t> cells(64, 0);
  cells[1 * 8 + 1] = 100;
  const auto grid = make_grid(cells);
  const std::vector<plan::Point2D> raw{{0.5, 0.5}, {2.5, 0.5}, {2.5, 2.5}};
  const auto path = plan::process_path(grid, raw, config(false, 1.5));
  ASSERT_EQ(path.size(), 5U);
  EXPECT_TRUE(contains(path, 2.0, 0.5));   // Arc length 1.5.
  EXPECT_TRUE(contains(path, 2.5, 0.5));   // Mandatory bend at arc length 2.
  EXPECT_TRUE(contains(path, 2.5, 1.5));   // Arc length 3, not reset at the bend.
  EXPECT_TRUE(contains(path, 2.5, 2.5));   // Endpoint remainder.
  EXPECT_EQ(plan::process_path(grid, raw, config(false, 100.0)).size(), 3U);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_LE(std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 1.5 + 1e-12);
    EXPECT_TRUE(
      plan::line_of_sight(
        grid, {path[i - 1].x, path[i - 1].y}, {path[i].x, path[i].y}));
  }
  const auto straight = plan::process_path(grid, {{0.5, 0.5}, {3.5, 0.5}}, config(false, 0.5));
  ASSERT_EQ(straight.size(), 7U);
  for (std::size_t i = 1; i < straight.size(); ++i) {
    EXPECT_NEAR(straight[i].x - straight[i - 1].x, 0.5, 1e-12);
  }
  EXPECT_NEAR(path[2].yaw, std::acos(-1.0) / 2.0, 1e-12);
  EXPECT_NEAR(path.back().yaw, path[path.size() - 2].yaw, 1e-12);
}

TEST(PathProcessing, UsesWorldMetricAndYawWithTranslatedRotatedOrigin)
{
  const auto grid = plan::PlanningGrid::from_inflated_grid(
    {8, 8, 0.5, 10.0, -3.0, std::acos(-1.0) / 2.0}, std::vector<std::int8_t>(64, 0), 1000);
  const auto a = grid.cell_to_world(0, 0), b = grid.cell_to_world(3, 0);
  const auto path = plan::process_path(
    grid, {{a.first, a.second}, {b.first, b.second}},
    config(true, 0.25));
  ASSERT_EQ(path.size(), 7U);
  EXPECT_NEAR(path.front().x, 9.75, 1e-12);
  EXPECT_NEAR(path.back().y, -1.25, 1e-12);
  for (const auto & point : path) {
    EXPECT_NEAR(point.yaw, std::acos(-1.0) / 2.0, 1e-12);
  }
}

TEST(PathProcessing, RejectsInvalidParametersInputAndExcessiveOutput)
{
  const auto grid = make_grid();
  const std::vector<plan::Point2D> raw{{0.5, 0.5}, {2.5, 0.5}, {2.5, 2.5}};
  auto invalid = config();
  invalid.resample_spacing = 0.0;
  EXPECT_THROW(plan::process_path(grid, raw, invalid), std::invalid_argument);
  invalid.resample_spacing = std::numeric_limits<double>::infinity();
  EXPECT_THROW(plan::process_path(grid, raw, invalid), std::invalid_argument);
  invalid = config();
  invalid.shortcut_max_lookahead = 0;
  EXPECT_THROW(plan::process_path(grid, raw, invalid), std::invalid_argument);
  invalid = config();
  invalid.max_output_points = 0;
  EXPECT_THROW(plan::process_path(grid, raw, invalid), std::invalid_argument);
  invalid = config();
  invalid.max_output_points = 2;
  EXPECT_THROW(plan::process_path(grid, raw, invalid), std::length_error);  // Bend adds a point.
  invalid = config(false, 1e-300);
  EXPECT_THROW(plan::process_path(grid, raw, invalid), std::length_error);
  EXPECT_THROW(plan::process_path(grid, {{NAN, 1.0}}, config()), std::invalid_argument);
  EXPECT_THROW(plan::process_path(grid, {{1.0, 1.0}, {9.0, 1.0}}, config()), std::invalid_argument);
  std::vector<std::int8_t> cells(64, 0);
  cells[1 * 8 + 1] = 100;
  const auto blocked = make_grid(cells);
  EXPECT_THROW(
    plan::process_path(blocked, {{0.5, 0.5}, {2.5, 2.5}}, config(true)),
    std::invalid_argument);
}

// Independent analytic closed-box intersection, not a grid traversal or point sampler.
bool reference_visible(const plan::PlanningGrid & grid, plan::Point2D a, plan::Point2D b)
{
  for (std::size_t y = 0; y < 8; ++y) {
    for (std::size_t x = 0; x < 8; ++x) {
      if (!grid.is_blocked(x, y)) {
        continue;
      }
      long double enter = 0.0L, leave = 1.0L;
      const auto slab = [&enter, &leave](long double start, long double end, long double lower) {
          const long double delta = end - start;
          if (delta == 0.0L) {
            return start >= lower && start <= lower + 1.0L;
          }
          long double first = (lower - start) / delta, last = (lower + 1.0L - start) / delta;
          if (first > last) {
            std::swap(first, last);
          }
          enter = std::max(enter, first);
          leave = std::min(leave, last);
          return enter <= leave;
        };
      if (slab(a.x, b.x, x) && slab(a.y, b.y, y)) {
        return false;
      }
    }
  }
  return true;
}

TEST(PathProcessing, SupercoverMatchesIndependentClosedBoxCollisionChecks)
{
  std::mt19937 random(425);
  for (int trial = 0; trial < 500; ++trial) {
    std::vector<std::int8_t> cells(64, 0);
    for (auto & cell : cells) {
      cell = random() % 6 == 0 ? 100 : 0;
    }
    const auto grid = make_grid(cells);
    const plan::Point2D a{0.25 + (random() % 30) * 0.25, 0.25 + (random() % 30) * 0.25};
    const plan::Point2D b{0.25 + (random() % 30) * 0.25, 0.25 + (random() % 30) * 0.25};
    const bool expected = reference_visible(grid, a, b);
    EXPECT_EQ(plan::line_of_sight(grid, a, b), expected) << "trial " << trial;
    EXPECT_EQ(plan::line_of_sight(grid, b, a), expected) << "reversed trial " << trial;
  }
}

TEST(PathProcessing, RandomAStarPathsRemainCollisionFreeAndNeverGrowInLength)
{
  std::mt19937 random(47);
  int successful = 0;
  for (int trial = 0; trial < 100; ++trial) {
    std::vector<std::int8_t> cells(64, 0);
    for (auto & cell : cells) {
      cell = random() % 8 == 0 ? 100 : 0;
    }
    cells.front() = cells.back() = 0;
    const auto grid = make_grid(cells);
    const auto search = plan::astar_search(&grid, {0, 0}, {7, 7});
    if (search.status != plan::PlanStatus::SUCCESS) {
      continue;
    }
    ++successful;
    std::vector<plan::Point2D> raw;
    for (const auto & cell : search.cells) {
      const auto point = grid.cell_to_world(cell.x, cell.y);
      raw.push_back({point.first, point.second});
    }
    const auto path = plan::process_path(grid, raw, config(true, 0.37));
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const plan::Point2D a{path[i - 1].x, path[i - 1].y}, b{path[i].x, path[i].y};
      EXPECT_TRUE(reference_visible(grid, a, b));
      const double distance = std::hypot(b.x - a.x, b.y - a.y);
      EXPECT_GT(distance, 0.0);
      EXPECT_LE(distance, 0.37 + 1e-12);
      EXPECT_NEAR(path[i - 1].yaw, std::atan2(b.y - a.y, b.x - a.x), 1e-12);
      length += distance;
    }
    EXPECT_LE(length, search.total_cost + 1e-10);
    EXPECT_DOUBLE_EQ(path.front().x, raw.front().x);
    EXPECT_DOUBLE_EQ(path.back().y, raw.back().y);
  }
  EXPECT_GT(successful, 20);
}

plan::PathProcessingConfig arc_config(double radius = 0.6, double minimum = 0.2)
{
  auto value = config(false, 0.1);
  value.enable_arc_smoothing = true;
  value.arc_radius = radius;
  value.arc_min_radius = minimum;
  return value;
}

TEST(PathProcessing, LeftAndRightFilletsAreTangentAndRespectRadiusAndDeviation)
{
  const auto grid = make_grid();
  for (double sign : {-1.0, 1.0}) {
    const double y = sign > 0.0 ? 1.5 : 5.5;
    const std::vector<plan::Point2D> raw{{1.5, y}, {5.5, y}, {5.5, y + sign * 4.0}};
    const auto cfg = arc_config();
    plan::PathProcessingStats stats;
    const auto path = plan::process_path(grid, raw, cfg, &stats);
    EXPECT_EQ(stats.rounded_corners, 1U);
    EXPECT_EQ(stats.retained_corners, 0U);
    EXPECT_FALSE(contains(path, 5.5, y));
    EXPECT_TRUE(contains(path, 4.9, y));  // Incoming tangency point.
    EXPECT_TRUE(contains(path, 5.5, y + sign * 0.6));  // Outgoing tangency point.
    EXPECT_DOUBLE_EQ(path.front().x, raw.front().x);
    EXPECT_DOUBLE_EQ(path.back().y, raw.back().y);
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto a = path[i - 1], b = path[i];
      EXPECT_TRUE(reference_visible(grid, {a.x, a.y}, {b.x, b.y}));
      EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y), cfg.resample_spacing + 1e-12);
      if (b.x > 4.9 && sign * (b.y - y) < 0.6) {
        const double radius = std::hypot(b.x - 4.9, b.y - (y + sign * 0.6));
        EXPECT_NEAR(radius, 0.6, cfg.arc_max_deviation + 1e-12);
      }
      if (i + 1 < path.size()) {
        const double change = std::remainder(path[i].yaw - path[i - 1].yaw, 2.0 * std::acos(-1.0));
        EXPECT_LE(std::abs(change), cfg.arc_max_angle + 1e-10);
      }
    }
  }
}

TEST(PathProcessing, CollisionSearchShrinksRadiusButNeverBelowMinimum)
{
  std::vector<std::int8_t> cells(64, 0);
  cells[2 * 8 + 4] = 100;
  const auto grid = make_grid(cells);
  const std::vector<plan::Point2D> raw{{0.5, 1.5}, {5.5, 1.5}, {5.5, 7.5}};
  plan::PathProcessingStats stats;
  const auto path = plan::process_path(grid, raw, arc_config(3.0, 0.5), &stats);
  ASSERT_EQ(stats.rounded_corners, 1U);
  EXPECT_EQ(stats.retained_corners, 0U);
  EXPECT_FALSE(contains(path, 5.5, 1.5));
  double entry_x = 0.0;
  for (const auto & point : path) {
    if (std::abs(point.y - 1.5) < 1e-12) {
      entry_x = point.x;
    }
  }
  EXPECT_LT(5.5 - entry_x, 3.0);
  EXPECT_GE(5.5 - entry_x, 0.5 - 1e-12);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(
      reference_visible(
        grid, {path[i - 1].x, path[i - 1].y}, {path[i].x, path[i].y}));
  }

  const auto fallback = plan::process_path(grid, raw, arc_config(3.0, 3.0), &stats);
  EXPECT_EQ(stats.rounded_corners, 0U);
  EXPECT_EQ(stats.retained_corners, 1U);
  EXPECT_TRUE(contains(fallback, 5.5, 1.5));
}

TEST(PathProcessing, ShortSegmentsAndReversalsRetainStopCorners)
{
  const auto grid = make_grid();
  plan::PathProcessingStats stats;
  const auto short_path = plan::process_path(
    grid, {{1.5, 1.5}, {3.5, 1.5}, {3.5, 2.0}, {5.5, 2.0}}, arc_config(0.6, 0.6), &stats);
  EXPECT_EQ(stats.rounded_corners, 0U);
  EXPECT_EQ(stats.retained_corners, 2U);
  EXPECT_TRUE(contains(short_path, 3.5, 1.5));
  EXPECT_TRUE(contains(short_path, 3.5, 2.0));
  const auto reverse = plan::process_path(
    grid, {{1.5, 1.5}, {3.5, 1.5}, {1.5, 1.5}}, arc_config(), &stats);
  EXPECT_EQ(stats.rounded_corners, 0U);
  EXPECT_EQ(stats.retained_corners, 1U);
  EXPECT_TRUE(contains(reverse, 3.5, 1.5));
}

TEST(PathProcessing, AdjacentFilletsShareSegmentWithoutOverlapOrDuplicatePoints)
{
  const auto grid = make_grid();
  plan::PathProcessingStats stats;
  const auto path = plan::process_path(
    grid, {{0.5, 0.5}, {3.5, 0.5}, {3.5, 1.5}, {6.5, 1.5}}, arc_config(1.0), &stats);
  EXPECT_EQ(stats.rounded_corners, 2U);
  EXPECT_EQ(stats.retained_corners, 0U);
  EXPECT_TRUE(contains(path, 3.5, 1.0));
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_GT(std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y), 1e-12);
    EXPECT_LE(path[i - 1].x, path[i].x + 1e-12);
    EXPECT_LE(path[i - 1].y, path[i].y + 1e-12);
  }
}

TEST(PathProcessing, SafeAndUnsafeCornersCanCoexistInOnePath)
{
  const auto grid = make_grid();
  plan::PathProcessingStats stats;
  const auto path = plan::process_path(
    grid, {{0.5, 0.5}, {3.5, 0.5}, {3.5, 4.5}, {3.8, 4.5}}, arc_config(0.6, 0.4), &stats);
  EXPECT_EQ(stats.rounded_corners, 1U);
  EXPECT_EQ(stats.retained_corners, 1U);
  EXPECT_FALSE(contains(path, 3.5, 0.5));
  EXPECT_TRUE(contains(path, 3.5, 4.5));
}

TEST(PathProcessing, ContinuousArcCollisionIsRejectedEvenWhenEveryOutputChordWouldBeFree)
{
  // A tiny obstacle intersects the first arc's midpoint, outside the four-chord polygon.
  const double resolution = 0.0002;
  const double angle = -std::acos(-1.0) / 2.0 + std::acos(-1.0) / 16.0;
  const plan::Point2D hit{0.13 + 0.05 * std::cos(angle), 0.07 + 0.05 * std::sin(angle)};
  std::vector<std::int8_t> cells(1000000, 0);
  cells[static_cast<std::size_t>(hit.y / resolution) * 1000 +
    static_cast<std::size_t>(hit.x / resolution)] = 100;
  const auto grid = plan::PlanningGrid::from_inflated_grid(
    {1000, 1000, resolution, 0.0, 0.0, 0.0}, std::move(cells), 1000000);
  const plan::Point2D first{0.13, 0.02};
  const plan::Point2D next{
    0.13 + 0.05 * std::cos(-std::acos(-1.0) / 2.0 + std::acos(-1.0) / 8.0),
    0.07 + 0.05 * std::sin(-std::acos(-1.0) / 2.0 + std::acos(-1.0) / 8.0)};
  ASSERT_TRUE(plan::line_of_sight(grid, first, next));
  ASSERT_FALSE(plan::line_of_sight(grid, hit, hit));
  auto cfg = arc_config(0.05, 0.05);
  cfg.resample_spacing = 0.1;
  cfg.arc_max_angle = 0.5;
  cfg.arc_max_deviation = 0.01;
  plan::PathProcessingStats stats;
  const auto path = plan::process_path(
    grid, {{0.02, 0.02}, {0.18, 0.02}, {0.18, 0.18}}, cfg, &stats);
  EXPECT_EQ(stats.rounded_corners, 0U);
  EXPECT_EQ(stats.retained_corners, 1U);
  EXPECT_TRUE(contains(path, 0.18, 0.02));
}

TEST(PathProcessing, FilletsRespectRotatedMapGeometryAndMapBoundary)
{
  const auto grid = plan::PlanningGrid::from_inflated_grid(
    {8, 8, 0.5, 10.0, -3.0, std::acos(-1.0) / 2.0}, std::vector<std::int8_t>(64, 0), 1000);
  plan::PathProcessingStats stats;
  const auto path = plan::process_path(
    grid, {{9.25, -2.25}, {9.25, -0.25}, {7.25, -0.25}}, arc_config(), &stats);
  EXPECT_EQ(stats.rounded_corners, 1U);
  EXPECT_NEAR(path.front().yaw, std::acos(-1.0) / 2.0, 1e-12);
  EXPECT_NEAR(path.back().yaw, std::acos(-1.0), 1e-12);
  for (std::size_t i = 1; i < path.size(); ++i) {
    EXPECT_TRUE(
      plan::line_of_sight(
        grid, {path[i - 1].x, path[i - 1].y}, {path[i].x, path[i].y}));
  }
  EXPECT_THROW(
    plan::process_path(grid, {{10.0, -3.0}, {9.0, -2.0}, {9.0, -1.0}}, arc_config()),
    std::invalid_argument);
}

TEST(PathProcessing, ArcConfigAndResourceLimitsAreValidated)
{
  const auto grid = make_grid();
  const std::vector<plan::Point2D> raw{{1.5, 1.5}, {5.5, 1.5}, {5.5, 5.5}};
  for (int invalid = 0; invalid < 8; ++invalid) {
    auto cfg = arc_config();
    switch (invalid) {
      case 0: cfg.arc_radius = 0.1; break;
      case 1: cfg.arc_min_radius = 0.0; break;
      case 2: cfg.arc_max_deviation = 0.0; break;
      case 3: cfg.arc_max_angle = 2.0; break;
      case 4: cfg.arc_radius_samples = 1; break;
      case 5: cfg.arc_radius_samples = 1001; break;
      case 6: cfg.arc_radius = std::numeric_limits<double>::infinity(); break;
      case 7: cfg.arc_max_angle = std::numeric_limits<double>::quiet_NaN(); break;
    }
    EXPECT_THROW(plan::process_path(grid, raw, cfg), std::invalid_argument);
  }
  auto cfg = arc_config();
  cfg.max_output_points = 10;
  EXPECT_THROW(plan::process_path(grid, raw, cfg), std::length_error);
  plan::PathProcessingStats stats{10, 10};
  EXPECT_TRUE(plan::process_path(grid, {}, arc_config(), &stats).empty());
  EXPECT_EQ(stats.rounded_corners, 0U);
  EXPECT_EQ(stats.retained_corners, 0U);
}

TEST(PathProcessing, RandomSmoothedAStarPathsStayFreeAndDoNotGrow)
{
  std::mt19937 random(962);
  std::size_t rounded = 0;
  int successful = 0;
  for (int trial = 0; trial < 100; ++trial) {
    std::vector<std::int8_t> cells(64, 0);
    for (auto & cell : cells) {
      cell = random() % 8 == 0 ? 100 : 0;
    }
    cells.front() = cells.back() = 0;
    const auto grid = make_grid(cells);
    const auto search = plan::astar_search(&grid, {0, 0}, {7, 7});
    if (search.status != plan::PlanStatus::SUCCESS) {
      continue;
    }
    ++successful;
    std::vector<plan::Point2D> raw;
    for (const auto & cell : search.cells) {
      const auto point = grid.cell_to_world(cell.x, cell.y);
      raw.push_back({point.first, point.second});
    }
    auto cfg = arc_config();
    cfg.enable_shortcut = true;
    plan::PathProcessingStats stats;
    const auto path = plan::process_path(grid, raw, cfg, &stats);
    rounded += stats.rounded_corners;
    double length = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const auto a = path[i - 1], b = path[i];
      EXPECT_TRUE(reference_visible(grid, {a.x, a.y}, {b.x, b.y}));
      const double step = std::hypot(b.x - a.x, b.y - a.y);
      EXPECT_GT(step, 0.0);
      EXPECT_LE(step, cfg.resample_spacing + 1e-12);
      length += step;
    }
    EXPECT_LE(length, search.total_cost + 1e-10);
    EXPECT_DOUBLE_EQ(path.front().x, raw.front().x);
    EXPECT_DOUBLE_EQ(path.back().y, raw.back().y);
  }
  EXPECT_GT(successful, 20);
  EXPECT_GT(rounded, 0U);
}
}  // namespace

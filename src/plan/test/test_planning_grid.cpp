// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/planning_grid.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

namespace
{
// Synthetic robot sizes test geometry, not the repository's real robot dimensions.
plan::GridConfig config()
{
  return {65, true, 0.1, 0.0, false, 10000};
}

plan::GridGeometry geometry(std::size_t width = 5, std::size_t height = 5)
{
  return {width, height, 1.0, 0.0, 0.0, 0.0};
}

TEST(PlanningGrid, ThresholdIsInclusiveAndRawDataIsPreserved)
{
  const std::vector<std::int8_t> input = {0, 64, 65, 100, -1};
  const plan::PlanningGrid grid(geometry(5, 1), input, config());
  EXPECT_EQ(grid.raw_grid(), input);
  EXPECT_EQ(grid.raw_obstacles(), (std::vector<std::uint8_t>{0, 0, 1, 1, 1}));
  EXPECT_EQ(grid.inflated_grid(), (std::vector<std::int8_t>{0, 0, 100, 100, 100}));
}

TEST(PlanningGrid, UnknownCanBeTraversableButRemainsUnknownInDebugGrid)
{
  auto settings = config();
  settings.unknown_is_obstacle = false;
  const plan::PlanningGrid grid(geometry(3, 1), {-1, 0, -1}, settings);
  EXPECT_EQ(grid.inflated_grid(), (std::vector<std::int8_t>{-1, 0, -1}));
  EXPECT_FALSE(grid.is_blocked(0, 0));
}

TEST(PlanningGrid, UnknownObstacleInflatesAndFreeUnknownMayBecomeBlocked)
{
  auto settings = config();
  settings.robot_radius = 1.0;
  std::vector<std::int8_t> input(25, 0);
  input[12] = -1;
  plan::PlanningGrid blocked(geometry(), input, settings);
  EXPECT_TRUE(blocked.is_blocked(3, 3));
  settings.unknown_is_obstacle = false;
  input[12] = 100;
  input[18] = -1;
  const plan::PlanningGrid allowed(geometry(), input, settings);
  EXPECT_EQ(allowed.inflated_grid()[18], 100);
  EXPECT_EQ(allowed.raw_grid()[18], -1);
}

TEST(PlanningGrid, EuclideanInflationIncludesCellExtentAndSafetyMargin)
{
  auto settings = config();
  settings.robot_radius = 0.8;
  settings.safety_margin = 0.2;
  std::vector<std::int8_t> input(49, 0);
  input[24] = 100;
  const plan::PlanningGrid grid(geometry(7, 7), input, settings);
  EXPECT_TRUE(grid.is_blocked(3, 3));
  EXPECT_TRUE(grid.is_blocked(4, 4));
  EXPECT_TRUE(grid.is_blocked(4, 3));
  EXPECT_FALSE(grid.is_blocked(5, 3));
  EXPECT_FALSE(grid.is_blocked(5, 5));
}

TEST(PlanningGrid, BoundaryInflationProtectsWholeRobot)
{
  auto settings = config();
  settings.robot_radius = 1.0;
  settings.inflate_map_boundary = true;
  const plan::PlanningGrid grid(geometry(), std::vector<std::int8_t>(25, 0), settings);
  EXPECT_TRUE(grid.is_blocked(0, 2));
  EXPECT_TRUE(grid.is_blocked(4, 2));
  EXPECT_FALSE(grid.is_blocked(2, 2));
  const plan::PlanningGrid narrow(geometry(2, 5), std::vector<std::int8_t>(10, 0), settings);
  for (auto value : narrow.inflated_grid()) {
    EXPECT_EQ(value, 100);
  }
}

TEST(PlanningGrid, EmptyAndFullyOccupiedMapsWork)
{
  const plan::PlanningGrid empty(geometry(), std::vector<std::int8_t>(25, 0), config());
  EXPECT_EQ(empty.inflated_grid(), empty.raw_grid());
  const plan::PlanningGrid full(geometry(), std::vector<std::int8_t>(25, 100), config());
  EXPECT_EQ(full.inflated_grid(), full.raw_grid());
}

TEST(PlanningGrid, TranslatedRotatedOriginRoundTripsCellCenters)
{
  auto meta = geometry(3, 2);
  meta.origin_x = 10.0;
  meta.origin_y = -3.0;
  meta.origin_yaw = std::acos(-1.0) / 2.0;
  meta.resolution = 0.5;
  const plan::PlanningGrid grid(meta, std::vector<std::int8_t>(6, 0), config());
  const auto first = grid.cell_to_world(0, 0);
  EXPECT_NEAR(first.first, 9.75, 1e-12);
  EXPECT_NEAR(first.second, -2.75, 1e-12);
  for (std::size_t y = 0; y < meta.height; ++y) {
    for (std::size_t x = 0; x < meta.width; ++x) {
      const auto point = grid.cell_to_world(x, y);
      std::size_t cx = 99, cy = 99;
      ASSERT_TRUE(grid.world_to_cell(point.first, point.second, cx, cy));
      EXPECT_EQ(cx, x);
      EXPECT_EQ(cy, y);
    }
  }
}

TEST(PlanningGrid, BoundsAndNonFiniteCoordinatesAreRejected)
{
  const plan::PlanningGrid grid(geometry(), std::vector<std::int8_t>(25, 0), config());
  std::size_t x = 7, y = 8;
  EXPECT_FALSE(grid.world_to_cell(-0.001, 0.0, x, y));
  EXPECT_FALSE(grid.world_to_cell(5.0, 0.0, x, y));
  EXPECT_FALSE(grid.world_to_cell(0.0, 5.0, x, y));
  EXPECT_FALSE(grid.world_to_cell(std::numeric_limits<double>::quiet_NaN(), 0.0, x, y));
  EXPECT_EQ(x, 7U);
  EXPECT_EQ(y, 8U);
  EXPECT_THROW(grid.cell_to_world(5, 0), std::out_of_range);
  EXPECT_THROW(grid.is_blocked(0, 5), std::out_of_range);
}

TEST(PlanningGrid, InvalidDataAndDimensionsAreRejected)
{
  EXPECT_THROW(plan::PlanningGrid(geometry(0, 1), {}, config()), std::invalid_argument);
  EXPECT_THROW(plan::PlanningGrid(geometry(2, 2), {0}, config()), std::invalid_argument);
  EXPECT_THROW(plan::PlanningGrid(geometry(1, 1), {-2}, config()), std::invalid_argument);
  EXPECT_THROW(plan::PlanningGrid(geometry(1, 1), {101}, config()), std::invalid_argument);
  auto meta = geometry(1, 1);
  meta.resolution = 0.0;
  EXPECT_THROW(plan::PlanningGrid(meta, {0}, config()), std::invalid_argument);
  meta.resolution = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(plan::PlanningGrid(meta, {0}, config()), std::invalid_argument);
  meta = geometry(std::numeric_limits<std::size_t>::max(), 2);
  EXPECT_THROW(plan::PlanningGrid(meta, {}, config()), std::invalid_argument);
}

TEST(PlanningGrid, InvalidParametersAreRejected)
{
  auto settings = config();
  settings.robot_radius = 0.0;
  EXPECT_THROW(plan::PlanningGrid::validate_config(settings), std::invalid_argument);
  settings = config();
  settings.safety_margin = -0.1;
  EXPECT_THROW(plan::PlanningGrid::validate_config(settings), std::invalid_argument);
  settings = config();
  settings.occupied_threshold = 101;
  EXPECT_THROW(plan::PlanningGrid::validate_config(settings), std::invalid_argument);
  settings = config();
  settings.max_cells = 0;
  EXPECT_THROW(plan::PlanningGrid::validate_config(settings), std::invalid_argument);
}

TEST(PlanningGrid, DistanceTransformMatchesBruteForceOnSeededMaps)
{
  std::mt19937 random(42);
  for (int trial = 0; trial < 30; ++trial) {
    auto meta = geometry(11, 8);
    meta.resolution = 0.2;
    auto settings = config();
    settings.robot_radius = 0.37;
    settings.safety_margin = 0.1;
    settings.unknown_is_obstacle = trial % 2 == 0;
    std::vector<std::int8_t> input(meta.width * meta.height, 0);
    for (auto & value : input) {
      const auto choice = random() % 12;
      value = choice == 0 ? 100 : (choice == 1 ? -1 : 0);
    }
    const plan::PlanningGrid grid(meta, input, settings);
    const double limit = (settings.robot_radius + settings.safety_margin) / meta.resolution +
      std::sqrt(2.0) / 2.0;
    for (std::size_t y = 0; y < meta.height; ++y) {
      for (std::size_t x = 0; x < meta.width; ++x) {
        bool expected = false;
        for (std::size_t j = 0; j < input.size(); ++j) {
          if (grid.raw_obstacles()[j]) {
            const double dx = static_cast<double>(x) - static_cast<double>(j % meta.width);
            const double dy = static_cast<double>(y) - static_cast<double>(j / meta.width);
            expected = expected || dx * dx + dy * dy <= limit * limit;
          }
        }
        EXPECT_EQ(grid.is_blocked(x, y), expected) << "trial=" << trial << ", x=" << x;
      }
    }
    EXPECT_EQ(grid.raw_grid(), input);
  }
}
}  // namespace

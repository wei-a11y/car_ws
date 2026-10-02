// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/astar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace
{
plan::PlanningGrid grid(std::size_t width, std::size_t height, std::vector<std::int8_t> cells)
{
  return plan::PlanningGrid::from_inflated_grid(
    {width, height, 0.5, 0.0, 0.0, 0.0}, std::move(cells), 10000);
}

TEST(AStar, StraightAndDiagonalCostsAreMetricAndOptimal)
{
  const auto map = grid(5, 5, std::vector<std::int8_t>(25, 0));
  const auto straight = plan::astar_search(&map, {0, 0}, {4, 0});
  ASSERT_EQ(straight.status, plan::PlanStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(straight.total_cost, 2.0);
  const auto diagonal = plan::astar_search(&map, {0, 0}, {4, 4});
  ASSERT_EQ(diagonal.status, plan::PlanStatus::SUCCESS);
  EXPECT_NEAR(diagonal.total_cost, 2.0 * std::sqrt(2.0), 1e-12);
  const auto mixed = plan::astar_search(&map, {0, 0}, {4, 2});
  ASSERT_EQ(mixed.status, plan::PlanStatus::SUCCESS);
  EXPECT_NEAR(mixed.total_cost, 1.0 + std::sqrt(2.0), 1e-12);
  EXPECT_EQ(mixed.cells.front(), (plan::Cell{0, 0}));
  EXPECT_EQ(mixed.cells.back(), (plan::Cell{4, 2}));
}

TEST(AStar, SameCellReturnsOnePointAndZeroCost)
{
  const auto map = grid(1, 1, {0});
  const auto result = plan::astar_search(&map, {0, 0}, {0, 0});
  ASSERT_EQ(result.status, plan::PlanStatus::SUCCESS);
  EXPECT_EQ(result.cells.size(), 1U);
  EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
}

TEST(AStar, BothOrthogonalNeighboursMustBeFree)
{
  const auto closed = grid(2, 2, {0, 100, 100, 0});
  EXPECT_EQ(plan::astar_search(&closed, {0, 0}, {1, 1}).status, plan::PlanStatus::NO_PATH);
  const auto one_wall = grid(2, 2, {0, 100, 0, 0});
  const auto result = plan::astar_search(&one_wall, {0, 0}, {1, 1});
  ASSERT_EQ(result.status, plan::PlanStatus::SUCCESS);
  EXPECT_EQ(result.cells.size(), 3U);
  EXPECT_DOUBLE_EQ(result.total_cost, 1.0);
}

TEST(AStar, InvalidEndpointsAreDistinguished)
{
  const auto map = grid(2, 2, {100, 0, 0, 100});
  EXPECT_EQ(plan::astar_search(&map, {0, 0}, {1, 0}).status, plan::PlanStatus::INVALID_START);
  EXPECT_EQ(plan::astar_search(&map, {2, 0}, {1, 0}).status, plan::PlanStatus::INVALID_START);
  EXPECT_EQ(plan::astar_search(&map, {1, 0}, {1, 1}).status, plan::PlanStatus::INVALID_GOAL);
  EXPECT_EQ(plan::astar_search(&map, {1, 0}, {2, 1}).status, plan::PlanStatus::INVALID_GOAL);
  EXPECT_EQ(plan::astar_search(nullptr, {0, 0}, {0, 0}).status, plan::PlanStatus::MAP_UNAVAILABLE);
}

TEST(AStar, WallReturnsNoPathAndFailuresDoNotReturnPartialPaths)
{
  const auto map = grid(3, 3, {0, 100, 0, 0, 100, 0, 0, 100, 0});
  const auto result = plan::astar_search(&map, {0, 1}, {2, 1});
  EXPECT_EQ(result.status, plan::PlanStatus::NO_PATH);
  EXPECT_TRUE(result.cells.empty());
  EXPECT_TRUE(std::isinf(result.total_cost));
}

TEST(AStar, Phase2OutputIsNotInflatedAgainAndUnknownPolicyIsPreserved)
{
  std::vector<std::int8_t> input(81, 0);
  input[40] = 100;
  const plan::PlanningGrid original({9, 9, 0.2, 0, 0, 0}, input,
    {65, true, 0.355, 0.05, false, 10000});
  const auto imported = plan::PlanningGrid::from_inflated_grid(
    original.geometry(), original.inflated_grid(), 10000);
  EXPECT_EQ(imported.inflated_grid(), original.inflated_grid());
  const auto allowed = grid(3, 1, {0, -1, 0});
  EXPECT_EQ(plan::astar_search(&allowed, {0, 0}, {2, 0}).status, plan::PlanStatus::SUCCESS);
  EXPECT_THROW(grid(1, 1, {65}), std::invalid_argument);
  EXPECT_THROW(grid(0, 1, {}), std::invalid_argument);
}

TEST(AStar, RotatedGridProducesCorrectWorldCellCentres)
{
  const auto map = plan::PlanningGrid::from_inflated_grid(
    {3, 2, 0.5, 10.0, -3.0, std::acos(-1.0) / 2.0}, std::vector<std::int8_t>(6, 0), 10000);
  const auto result = plan::astar_search(&map, {0, 0}, {2, 1});
  ASSERT_EQ(result.status, plan::PlanStatus::SUCCESS);
  const auto point = map.cell_to_world(result.cells.back().x, result.cells.back().y);
  EXPECT_NEAR(point.first, 9.25, 1e-12);
  EXPECT_NEAR(point.second, -1.75, 1e-12);
}

// Dense O(V^2) Dijkstra, independent of A*'s priority queue and heuristic.
double reference_cost(const plan::PlanningGrid & map)
{
  const auto & meta = map.geometry();
  const auto size = meta.width * meta.height;
  std::vector<double> cost(size, std::numeric_limits<double>::infinity());
  std::vector<bool> visited(size, false);
  cost[0] = 0.0;
  for (std::size_t iteration = 0; iteration < size; ++iteration) {
    std::size_t best = size;
    for (std::size_t i = 0; i < size; ++i) {
      if (!visited[i] && (best == size || cost[i] < cost[best])) {
        best = i;
      }
    }
    if (best == size || !std::isfinite(cost[best])) {
      break;
    }
    visited[best] = true;
    const auto x = best % meta.width, y = best / meta.width;
    for (std::size_t next = 0; next < size; ++next) {
      const auto nx = next % meta.width, ny = next / meta.width;
      const auto dx = std::abs(static_cast<double>(nx) - static_cast<double>(x));
      const auto dy = std::abs(static_cast<double>(ny) - static_cast<double>(y));
      if ((dx == 0.0 && dy == 0.0) || dx > 1.0 || dy > 1.0 || map.is_blocked(nx, ny)) {
        continue;
      }
      if (dx != 0.0 && dy != 0.0 && (map.is_blocked(nx, y) || map.is_blocked(x, ny))) {
        continue;
      }
      cost[next] = std::min(cost[next], cost[best] + meta.resolution * std::hypot(dx, dy));
    }
  }
  return cost.back();
}

TEST(AStar, OptimalityAndLegalStepsMatchIndependentDijkstra)
{
  std::mt19937 random(123);
  for (int trial = 0; trial < 30; ++trial) {
    std::vector<std::int8_t> cells(48, 0);
    for (auto & cell : cells) {
      cell = random() % 5 == 0 ? 100 : 0;
    }
    cells.front() = cells.back() = 0;
    const auto map = grid(8, 6, cells);
    const auto result = plan::astar_search(&map, {0, 0}, {7, 5});
    const double expected = reference_cost(map);
    if (!std::isfinite(expected)) {
      EXPECT_EQ(result.status, plan::PlanStatus::NO_PATH);
      continue;
    }
    ASSERT_EQ(result.status, plan::PlanStatus::SUCCESS);
    EXPECT_NEAR(result.total_cost, expected, 1e-12);
    for (std::size_t i = 1; i < result.cells.size(); ++i) {
      const auto & a = result.cells[i - 1];
      const auto & b = result.cells[i];
      EXPECT_FALSE(map.is_blocked(b.x, b.y));
      const auto dx = std::abs(static_cast<double>(b.x) - static_cast<double>(a.x));
      const auto dy = std::abs(static_cast<double>(b.y) - static_cast<double>(a.y));
      EXPECT_LE(dx, 1.0);
      EXPECT_LE(dy, 1.0);
      EXPECT_GT(dx + dy, 0.0);
      if (dx != 0.0 && dy != 0.0) {
        EXPECT_FALSE(map.is_blocked(b.x, a.y));
        EXPECT_FALSE(map.is_blocked(a.x, b.y));
      }
    }
  }
}
}  // namespace

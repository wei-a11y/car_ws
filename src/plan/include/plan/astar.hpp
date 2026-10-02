// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__ASTAR_HPP_
#define PLAN__ASTAR_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "plan/planning_grid.hpp"

namespace plan
{
enum class PlanStatus
{
  SUCCESS,
  INVALID_START,
  INVALID_GOAL,
  NO_PATH,
  MAP_UNAVAILABLE,
  TF_UNAVAILABLE
};

const char * status_name(PlanStatus status);

struct Cell
{
  std::size_t x;
  std::size_t y;
  bool operator==(const Cell & other) const {return x == other.x && y == other.y;}
};

struct PlanResult
{
  PlanStatus status;
  std::vector<Cell> cells = {};
  double total_cost = std::numeric_limits<double>::infinity();  // metres
};

// 8-connected A*, octile heuristic; both orthogonal neighbours must be free for diagonals.
// A null grid returns MAP_UNAVAILABLE. TF_UNAVAILABLE belongs to the ROS adapter.
PlanResult astar_search(const PlanningGrid * grid, Cell start, Cell goal);
}  // namespace plan
#endif  // PLAN__ASTAR_HPP_

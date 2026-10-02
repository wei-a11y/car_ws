// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace plan
{
const char * status_name(PlanStatus status)
{
  switch (status) {
    case PlanStatus::SUCCESS: return "SUCCESS";
    case PlanStatus::INVALID_START: return "INVALID_START";
    case PlanStatus::INVALID_GOAL: return "INVALID_GOAL";
    case PlanStatus::NO_PATH: return "NO_PATH";
    case PlanStatus::MAP_UNAVAILABLE: return "MAP_UNAVAILABLE";
    case PlanStatus::TF_UNAVAILABLE: return "TF_UNAVAILABLE";
  }
  return "NO_PATH";
}

namespace
{
struct Entry
{
  double f;
  double g;
  std::size_t index;
};

struct HigherCost
{
  bool operator()(const Entry & a, const Entry & b) const
  {
    return a.f == b.f ? a.index > b.index : a.f > b.f;
  }
};

double heuristic(Cell from, Cell goal, double resolution)
{
  const double dx = std::abs(static_cast<double>(from.x) - static_cast<double>(goal.x));
  const double dy = std::abs(static_cast<double>(from.y) - static_cast<double>(goal.y));
  return (std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy)) * resolution;
}
}  // namespace

PlanResult astar_search(const PlanningGrid * grid, Cell start, Cell goal)
{
  if (grid == nullptr) {
    return {PlanStatus::MAP_UNAVAILABLE};
  }
  const auto & meta = grid->geometry();
  if (start.x >= meta.width || start.y >= meta.height || grid->is_blocked(start.x, start.y)) {
    return {PlanStatus::INVALID_START};
  }
  if (goal.x >= meta.width || goal.y >= meta.height || grid->is_blocked(goal.x, goal.y)) {
    return {PlanStatus::INVALID_GOAL};
  }
  if (start == goal) {
    return {PlanStatus::SUCCESS, {start}, 0.0};
  }
  const std::size_t count = grid->inflated_grid().size();
  const std::size_t start_id = start.y * meta.width + start.x;
  const std::size_t goal_id = goal.y * meta.width + goal.x;
  std::vector<double> costs(count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parents(count, count);
  std::vector<std::uint8_t> closed(count, 0);
  std::priority_queue<Entry, std::vector<Entry>, HigherCost> open;
  costs[start_id] = 0.0;
  open.push({heuristic(start, goal, meta.resolution), 0.0, start_id});
  constexpr std::array<std::pair<int, int>, 8> kNeighbours = {
    {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}};
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (closed[current.index] || current.g > costs[current.index]) {
      continue;
    }
    if (current.index == goal_id) {
      std::vector<Cell> path;
      for (std::size_t id = goal_id; ; id = parents[id]) {
        path.push_back({id % meta.width, id / meta.width});
        if (id == start_id) {
          break;
        }
      }
      std::reverse(path.begin(), path.end());
      return {PlanStatus::SUCCESS, std::move(path), costs[goal_id]};
    }
    closed[current.index] = 1;
    const Cell from{current.index % meta.width, current.index / meta.width};
    for (const auto & offset : kNeighbours) {
      const int dx = offset.first, dy = offset.second;
      if ((dx < 0 && from.x == 0) || (dx > 0 && from.x + 1 >= meta.width) ||
        (dy < 0 && from.y == 0) || (dy > 0 && from.y + 1 >= meta.height))
      {
        continue;
      }
      const Cell next{
        dx < 0 ? from.x - 1 : from.x + dx, dy < 0 ? from.y - 1 : from.y + dy};
      const std::size_t id = next.y * meta.width + next.x;
      if (closed[id] || grid->is_blocked(next.x, next.y)) {
        continue;
      }
      const bool diagonal = dx != 0 && dy != 0;
      if (diagonal && (grid->is_blocked(next.x, from.y) || grid->is_blocked(from.x, next.y))) {
        continue;
      }
      const double candidate = current.g + meta.resolution * (diagonal ? std::sqrt(2.0) : 1.0);
      if (candidate < costs[id]) {
        costs[id] = candidate;
        parents[id] = current.index;
        open.push({candidate + heuristic(next, goal, meta.resolution), candidate, id});
      }
    }
  }
  return {PlanStatus::NO_PATH};
}
}  // namespace plan

// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/planning_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace plan
{
namespace
{
void validate_geometry(const GridGeometry & geometry, std::size_t count, std::size_t max_cells)
{
  if (max_cells == 0 || geometry.width == 0 || geometry.height == 0 ||
    geometry.width > max_cells / geometry.height ||
    count != geometry.width * geometry.height ||
    !std::isfinite(geometry.resolution) || geometry.resolution <= 0.0 ||
    !std::isfinite(geometry.origin_x) || !std::isfinite(geometry.origin_y) ||
    !std::isfinite(geometry.origin_yaw) ||
    !std::isfinite(geometry.resolution * static_cast<double>(geometry.width)) ||
    !std::isfinite(geometry.resolution * static_cast<double>(geometry.height)))
  {
    throw std::invalid_argument("Invalid grid geometry, cell count or occupancy data size");
  }
}

// Lower envelope of parabolas: exact squared Euclidean distance in O(n).
std::vector<double> distance_transform(const std::vector<double> & input)
{
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> output(input.size(), infinity);
  std::vector<std::size_t> sites(input.size());
  std::vector<double> boundaries(input.size() + 1);
  std::size_t count = 0;
  for (std::size_t q = 0; q < input.size(); ++q) {
    if (!std::isfinite(input[q])) {
      continue;
    }
    double crossing = -infinity;
    while (count != 0) {
      const std::size_t p = sites[count - 1];
      const double qd = static_cast<double>(q);
      const double pd = static_cast<double>(p);
      crossing = ((input[q] + qd * qd) - (input[p] + pd * pd)) / (2.0 * (qd - pd));
      if (crossing > boundaries[count - 1]) {
        break;
      }
      --count;
    }
    sites[count] = q;
    boundaries[count] = count == 0 ? -infinity : crossing;
    ++count;
    boundaries[count] = infinity;
  }
  if (count == 0) {
    return output;
  }
  std::size_t k = 0;
  for (std::size_t q = 0; q < input.size(); ++q) {
    while (k + 1 < count && boundaries[k + 1] < static_cast<double>(q)) {
      ++k;
    }
    const double delta = static_cast<double>(q) - static_cast<double>(sites[k]);
    output[q] = delta * delta + input[sites[k]];
  }
  return output;
}
}  // namespace

void PlanningGrid::validate_config(const GridConfig & config)
{
  if (config.occupied_threshold < 1 || config.occupied_threshold > 100 ||
    !std::isfinite(config.robot_radius) || config.robot_radius <= 0.0 ||
    !std::isfinite(config.safety_margin) || config.safety_margin < 0.0 ||
    !std::isfinite(config.robot_radius + config.safety_margin) || config.max_cells == 0)
  {
    throw std::invalid_argument("Invalid threshold, robot radius, safety margin or max_cells");
  }
}

PlanningGrid::PlanningGrid(
  GridGeometry geometry, std::vector<std::int8_t> occupancy, GridConfig config)
: geometry_(geometry), raw_(std::move(occupancy))
{
  validate_config(config);
  validate_geometry(geometry, raw_.size(), config.max_cells);
  obstacles_.resize(raw_.size());
  for (std::size_t i = 0; i < raw_.size(); ++i) {
    if (raw_[i] < -1 || raw_[i] > 100) {
      throw std::invalid_argument("Occupancy values must be -1 or in [0, 100]");
    }
    obstacles_[i] = raw_[i] == -1 ? config.unknown_is_obstacle :
      raw_[i] >= config.occupied_threshold;
  }

  std::vector<double> distances(raw_.size(), std::numeric_limits<double>::infinity());
  std::vector<double> line(geometry.width);
  for (std::size_t y = 0; y < geometry.height; ++y) {
    for (std::size_t x = 0; x < geometry.width; ++x) {
      line[x] = obstacles_[y * geometry.width + x] ? 0.0 :
        std::numeric_limits<double>::infinity();
    }
    const auto row = distance_transform(line);
    std::copy(row.begin(), row.end(), distances.begin() + y * geometry.width);
  }
  line.resize(geometry.height);
  for (std::size_t x = 0; x < geometry.width; ++x) {
    for (std::size_t y = 0; y < geometry.height; ++y) {
      line[y] = distances[y * geometry.width + x];
    }
    const auto column = distance_transform(line);
    for (std::size_t y = 0; y < geometry.height; ++y) {
      distances[y * geometry.width + x] = column[y];
    }
  }

  const double radius = config.robot_radius + config.safety_margin;
  // Occupied cells are squares, not points. Bound each by its circumscribed circle.
  const double cell_radius = radius / geometry.resolution + std::sqrt(2.0) / 2.0;
  if (!std::isfinite(cell_radius) || !std::isfinite(cell_radius * cell_radius)) {
    throw std::invalid_argument("Inflation radius in cells is not finite");
  }
  inflated_.resize(raw_.size());
  for (std::size_t y = 0; y < geometry.height; ++y) {
    for (std::size_t x = 0; x < geometry.width; ++x) {
      const std::size_t i = y * geometry.width + x;
      const double edge_distance = geometry.resolution * std::min(
        {static_cast<double>(x) + 0.5, static_cast<double>(y) + 0.5,
          static_cast<double>(geometry.width - x) - 0.5,
          static_cast<double>(geometry.height - y) - 0.5});
      const bool blocked = distances[i] <= cell_radius * cell_radius ||
        (config.inflate_map_boundary && edge_distance <= radius);
      inflated_[i] = blocked ? 100 : (raw_[i] == -1 ? -1 : 0);
    }
  }
}

PlanningGrid PlanningGrid::from_inflated_grid(
  GridGeometry geometry, std::vector<std::int8_t> cells, std::size_t max_cells)
{
  return PlanningGrid(geometry, std::move(cells), max_cells);
}

PlanningGrid::PlanningGrid(
  GridGeometry geometry, std::vector<std::int8_t> cells, std::size_t max_cells)
: geometry_(geometry), raw_(std::move(cells))
{
  validate_geometry(geometry, raw_.size(), max_cells);
  obstacles_.resize(raw_.size());
  for (std::size_t i = 0; i < raw_.size(); ++i) {
    if (raw_[i] != -1 && raw_[i] != 0 && raw_[i] != 100) {
      throw std::invalid_argument("Inflated grid values must be -1, 0 or 100");
    }
    obstacles_[i] = raw_[i] == 100;
  }
  inflated_ = raw_;
}

std::size_t PlanningGrid::index(std::size_t x, std::size_t y) const
{
  if (x >= geometry_.width || y >= geometry_.height) {
    throw std::out_of_range("Cell is outside grid");
  }
  return y * geometry_.width + x;
}

bool PlanningGrid::is_blocked(std::size_t x, std::size_t y) const
{
  return inflated_[index(x, y)] == 100;
}

std::pair<double, double> PlanningGrid::cell_to_world(std::size_t x, std::size_t y) const
{
  index(x, y);
  const double local_x = (static_cast<double>(x) + 0.5) * geometry_.resolution;
  const double local_y = (static_cast<double>(y) + 0.5) * geometry_.resolution;
  const double c = std::cos(geometry_.origin_yaw);
  const double s = std::sin(geometry_.origin_yaw);
  return {geometry_.origin_x + c * local_x - s * local_y,
    geometry_.origin_y + s * local_x + c * local_y};
}

bool PlanningGrid::world_to_cell(
  double x, double y, std::size_t & cell_x, std::size_t & cell_y) const
{
  if (!std::isfinite(x) || !std::isfinite(y)) {
    return false;
  }
  const double c = std::cos(geometry_.origin_yaw);
  const double s = std::sin(geometry_.origin_yaw);
  const double dx = x - geometry_.origin_x;
  const double dy = y - geometry_.origin_y;
  const double gx = (c * dx + s * dy) / geometry_.resolution;
  const double gy = (-s * dx + c * dy) / geometry_.resolution;
  if (!std::isfinite(gx) || !std::isfinite(gy) || gx < 0.0 || gy < 0.0 ||
    gx >= static_cast<double>(geometry_.width) || gy >= static_cast<double>(geometry_.height))
  {
    return false;
  }
  cell_x = static_cast<std::size_t>(std::floor(gx));
  cell_y = static_cast<std::size_t>(std::floor(gy));
  return true;
}
}  // namespace plan

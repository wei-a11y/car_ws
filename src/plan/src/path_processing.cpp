// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "plan/path_processing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace plan
{
namespace
{
// Numerical roundoff only, not a tunable collision clearance or simplification distance.
double roundoff(double value)
{
  return 64.0 * std::numeric_limits<double>::epsilon() * std::max(1.0, std::abs(value));
}

double snap_grid_line(double value)
{
  const double rounded = std::round(value);
  return std::abs(value - rounded) <= roundoff(value) ? rounded : value;
}

Point2D grid_coordinates(const GridGeometry & geometry, Point2D point)
{
  const double dx = point.x - geometry.origin_x, dy = point.y - geometry.origin_y;
  const double c = std::cos(geometry.origin_yaw), s = std::sin(geometry.origin_yaw);
  return {snap_grid_line((c * dx + s * dy) / geometry.resolution),
    snap_grid_line((-s * dx + c * dy) / geometry.resolution)};
}

bool same_point(Point2D a, Point2D b)
{
  return a.x == b.x && a.y == b.y;
}

bool collinear_forward(Point2D a, Point2D b, Point2D c)
{
  const double ux = b.x - a.x, uy = b.y - a.y;
  const double vx = c.x - b.x, vy = c.y - b.y;
  const double scale = std::hypot(ux, uy) * std::hypot(vx, vy);
  return std::isfinite(scale) && std::abs(ux * vy - uy * vx) <= roundoff(1.0) * scale &&
         ux * vx + uy * vy > 0.0;
}
}  // namespace

void validate_path_processing_config(const PathProcessingConfig & config)
{
  if (!std::isfinite(config.resample_spacing) || config.resample_spacing <= 0.0 ||
    config.shortcut_max_lookahead == 0 || config.max_output_points == 0)
  {
    throw std::invalid_argument("Path spacing must be finite/positive and limits must be positive");
  }
}

bool line_of_sight(const PlanningGrid & grid, Point2D from, Point2D to)
{
  const auto & geometry = grid.geometry();
  const Point2D a = grid_coordinates(geometry, from), b = grid_coordinates(geometry, to);
  // Check all closed cells containing a point, including both sides of grid lines.
  const auto free_point = [&grid, &geometry](Point2D point) {
      point.x = snap_grid_line(point.x);
      point.y = snap_grid_line(point.y);
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || point.x < 0.0 || point.y < 0.0 ||
        point.x >= static_cast<double>(geometry.width) ||
        point.y >= static_cast<double>(geometry.height))
      {
        return false;
      }
      const auto x = static_cast<std::int64_t>(std::floor(point.x));
      const auto y = static_cast<std::int64_t>(std::floor(point.y));
      const auto low_x = x - (point.x == std::floor(point.x) ? 1 : 0);
      const auto low_y = y - (point.y == std::floor(point.y) ? 1 : 0);
      for (auto cy = low_y; cy <= y; ++cy) {
        for (auto cx = low_x; cx <= x; ++cx) {
          if (cx < 0 || cy < 0 || grid.is_blocked(cx, cy)) {
            return false;
          }
        }
      }
      return true;
    };
  if (!free_point(a) || !free_point(b)) {
    return false;
  }
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double infinity = std::numeric_limits<double>::infinity();
  const double delta_x = dx == 0.0 ? infinity : 1.0 / std::abs(dx);
  const double delta_y = dy == 0.0 ? infinity : 1.0 / std::abs(dy);
  double next_x = dx == 0.0 ? infinity :
    ((dx > 0.0 ? std::floor(a.x) + 1.0 : std::ceil(a.x) - 1.0) - a.x) / dx;
  double next_y = dy == 0.0 ? infinity :
    ((dy > 0.0 ? std::floor(a.y) + 1.0 : std::ceil(a.y) - 1.0) - a.y) / dy;
  const auto at = [a, dx, dy](double t) -> Point2D {return {a.x + dx * t, a.y + dy * t};};
  double previous = 0.0;
  // Visit every grid-line event plus each intervening open interval; never sample-skip a cell.
  for (std::size_t step = 0; step <= geometry.width + geometry.height; ++step) {
    const double next = std::min({next_x, next_y, 1.0});
    if (!free_point(at((previous + next) / 2.0)) || !free_point(at(next))) {
      return false;
    }
    if (next == 1.0) {
      return true;
    }
    if (next_x <= next) {
      next_x += delta_x;
    }
    if (next_y <= next) {
      next_y += delta_y;
    }
    previous = next;
  }
  return false;
}

std::vector<PathPoint2D> process_path(
  const PlanningGrid & grid, const std::vector<Point2D> & raw,
  const PathProcessingConfig & config)
{
  validate_path_processing_config(config);
  std::vector<Point2D> simplified;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    if (!line_of_sight(grid, i == 0 ? raw[i] : raw[i - 1], raw[i])) {
      throw std::invalid_argument("Raw path contains an invalid or colliding segment");
    }
    if (!simplified.empty() && same_point(simplified.back(), raw[i])) {
      continue;
    }
    while (simplified.size() >= 2 &&
      collinear_forward(simplified[simplified.size() - 2], simplified.back(), raw[i]))
    {
      simplified.pop_back();
    }
    simplified.push_back(raw[i]);
  }
  // Validate merged segments once, not each growing collinear prefix (which would be O(n^2)).
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    if (!line_of_sight(grid, simplified[i - 1], simplified[i])) {
      throw std::invalid_argument("Collinear simplification failed collision validation");
    }
  }
  if (config.enable_shortcut && simplified.size() > 2) {
    std::vector<Point2D> shortened{simplified.front()};
    for (std::size_t current = 0; current + 1 < simplified.size(); ) {
      std::size_t next = current + std::min(
        config.shortcut_max_lookahead, simplified.size() - 1 - current);
      while (next > current + 1 && !line_of_sight(grid, simplified[current], simplified[next])) {
        --next;
      }
      shortened.push_back(simplified[next]);
      current = next;
    }
    simplified = std::move(shortened);
  }
  double total_length = 0.0;
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    total_length += std::hypot(
      simplified[i].x - simplified[i - 1].x, simplified[i].y - simplified[i - 1].y);
  }
  if (!std::isfinite(total_length) ||
    std::ceil(total_length / config.resample_spacing) + 1.0 > config.max_output_points)
  {
    throw std::length_error("Resampling would exceed max_output_points");
  }
  std::vector<PathPoint2D> output;
  const auto append = [&output, &config](Point2D point) {
      if (!output.empty() && same_point({output.back().x, output.back().y}, point)) {
        return;
      }
      if (output.size() >= config.max_output_points) {
        throw std::length_error("Resampling including bends exceeds max_output_points");
      }
      output.push_back({point.x, point.y, 0.0});
    };
  if (!simplified.empty()) {
    append(simplified.front());
  }
  double travelled = 0.0;
  std::size_t sample_index = 1;
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    const auto a = simplified[i - 1], b = simplified[i];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double end = travelled + length;
    double next_sample = sample_index * config.resample_spacing;
    while (next_sample < end - roundoff(end)) {
      const double ratio = (next_sample - travelled) / length;
      append({a.x + (b.x - a.x) * ratio, a.y + (b.y - a.y) * ratio});
      next_sample = ++sample_index * config.resample_spacing;
    }
    if (std::abs(next_sample - end) <= roundoff(end)) {
      ++sample_index;
    }
    // A bend is mandatory: connecting samples on either side would create a new corner chord.
    append(b);
    travelled = end;
  }
  for (std::size_t i = 0; i < output.size(); ++i) {
    if (i + 1 < output.size()) {
      if (!line_of_sight(grid, {output[i].x, output[i].y}, {output[i + 1].x, output[i + 1].y})) {
        throw std::invalid_argument("Processed path failed final collision validation");
      }
      output[i].yaw = std::atan2(output[i + 1].y - output[i].y, output[i + 1].x - output[i].x);
    } else if (i > 0) {
      output[i].yaw = output[i - 1].yaw;
    }
  }
  return output;
}
}  // namespace plan

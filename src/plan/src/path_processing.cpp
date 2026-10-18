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

double distance(Point2D a, Point2D b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

// Test a closed line segment against a closed axis-aligned box (slab method).
bool intersects_box(Point2D a, Point2D b, double x0, double y0, double x1, double y1)
{
  double enter = 0.0, leave = 1.0;
  const auto slab = [&enter, &leave](double start, double end, double low, double high) {
      const double delta = end - start;
      if (delta == 0.0) {
        return start >= low && start <= high;
      }
      double first = (low - start) / delta, last = (high - start) / delta;
      if (first > last) {
        std::swap(first, last);
      }
      enter = std::max(enter, first);
      leave = std::min(leave, last);
      return enter <= leave;
    };
  return slab(a.x, b.x, x0, x1) && slab(a.y, b.y, y0, y1);
}

bool arc_chord_is_free(const PlanningGrid & grid, Point2D from, Point2D to, double sagitta)
{
  const auto & geometry = grid.geometry();
  const auto a = grid_coordinates(geometry, from), b = grid_coordinates(geometry, to);
  // The entire minor arc is within sagitta of its chord. Expanding closed cell boxes
  // by that bound rejects both chord and continuous-arc collisions, including touches.
  // This is a discretization-error envelope, not a second robot-radius inflation.
  const double scale = std::max({std::abs(a.x), std::abs(a.y), std::abs(b.x), std::abs(b.y)});
  const double padding = sagitta / geometry.resolution + roundoff(scale);
  const double x0 = std::min(a.x, b.x) - padding, x1 = std::max(a.x, b.x) + padding;
  const double y0 = std::min(a.y, b.y) - padding, y1 = std::max(a.y, b.y) + padding;
  if (!std::isfinite(x0) || !std::isfinite(x1) || !std::isfinite(y0) || !std::isfinite(y1) ||
    x0 <= 0.0 || y0 <= 0.0 || x1 >= geometry.width || y1 >= geometry.height)
  {
    return false;
  }
  for (auto y = static_cast<std::size_t>(std::floor(y0)); y <= std::floor(y1); ++y) {
    for (auto x = static_cast<std::size_t>(std::floor(x0)); x <= std::floor(x1); ++x) {
      if (grid.is_blocked(x, y) && intersects_box(
          a, b, x - padding, y - padding, x + 1.0 + padding, y + 1.0 + padding))
      {
        return false;
      }
    }
  }
  return true;
}

struct Fillet
{
  std::vector<PathPoint2D> points;
  double trim = 0.0;
  bool space_limited = false;
};

Point2D xy(const PathPoint2D & p) {return {p.x, p.y};}

Fillet choose_fillet(
  const PlanningGrid & grid, Point2D a, Point2D b, Point2D c, double available,
  const PathProcessingConfig & config, double previous_radius = 0.0)
{
  const double incoming = distance(a, b), outgoing = distance(b, c);
  const Point2D u{(b.x - a.x) / incoming, (b.y - a.y) / incoming};
  const Point2D v{(c.x - b.x) / outgoing, (c.y - b.y) / outgoing};
  const double turn = std::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y);
  const double tangent_factor = std::tan(std::abs(turn) / 2.0);
  const double maximum = config.arc_radius_max > 0.0 ? config.arc_radius_max : config.arc_radius;
  const double upper = std::min(maximum, available / tangent_factor);
  Fillet best;
  best.space_limited = !std::isfinite(tangent_factor) || tangent_factor <= 0.0 ||
    std::abs(turn) <= roundoff(1.0) || upper < config.arc_min_radius;
  if (best.space_limited) {
    return best;
  }
  std::vector<double> radii{std::clamp(config.arc_radius, config.arc_min_radius, upper)};
  if (previous_radius >= config.arc_min_radius && previous_radius <= upper) {
    radii.push_back(previous_radius);
  }
  for (std::size_t trial = 0; trial < config.arc_radius_samples; ++trial) {
    const double ratio = static_cast<double>(trial) / (config.arc_radius_samples - 1);
    radii.push_back((1.0 - ratio) * upper + ratio * config.arc_min_radius);
  }
  double best_cost = std::numeric_limits<double>::infinity();
  for (double radius : radii) {
    const double cost = config.arc_radius_weight * std::pow(radius - config.arc_radius, 2) +
      config.arc_curvature_weight / radius;
    if (cost >= best_cost) {
      continue;
    }
    const double trim = radius * tangent_factor;
    const Point2D entry{b.x - trim * u.x, b.y - trim * u.y};
    const Point2D exit{b.x + trim * v.x, b.y + trim * v.y};
    const double sign = turn > 0.0 ? 1.0 : -1.0;
    const Point2D centre{entry.x - sign * radius * u.y, entry.y + sign * radius * u.x};
    const double start_angle = std::atan2(entry.y - centre.y, entry.x - centre.x);
    const double heading = std::atan2(u.y, u.x);
    const double deviation_angle = 4.0 * std::asin(
      std::sqrt(std::min(0.5, config.arc_max_deviation / radius / 2.0)));
    const double step = std::min(
      {config.arc_max_angle, deviation_angle, config.resample_spacing / radius});
    const double count = std::ceil(std::abs(turn) / step);
    if (!std::isfinite(count) || count + 1.0 > config.max_output_points) {
      throw std::length_error("Arc sampling exceeds max_output_points");
    }
    const auto segments = static_cast<std::size_t>(count);
    const double delta = turn / segments;
    const double sagitta = 2.0 * radius * std::pow(std::sin(delta / 4.0), 2);
    std::vector<PathPoint2D> arc{{entry.x, entry.y, 0.0, heading, sign / radius, false}};
    bool free = line_of_sight(grid, a, entry) && line_of_sight(grid, exit, c);
    for (std::size_t j = 1; free && j <= segments; ++j) {
      const double angle = start_angle + delta * j;
      const Point2D point = j == segments ? exit : Point2D{
        centre.x + radius * std::cos(angle), centre.y + radius * std::sin(angle)};
      free = arc_chord_is_free(grid, xy(arc.back()), point, sagitta);
      arc.push_back(
        {point.x, point.y, 0.0, heading + delta * j,
          j == segments ? 0.0 : sign / radius, false});
    }
    if (free) {
      best.points = std::move(arc);
      best.trim = trim;
      best_cost = cost;
    }
  }
  return best;
}

std::vector<PathPoint2D> round_corners(
  const PlanningGrid & grid, const std::vector<Point2D> & points,
  const PathProcessingConfig & config, PathProcessingStats & stats)
{
  std::vector<Fillet> fillets(points.size());
  if (config.enable_arc_smoothing) {
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
      const double available = std::min(
        distance(points[i - 1], points[i]) * (i == 1 ? 1.0 : 0.5),
        distance(points[i], points[i + 1]) * (i + 2 == points.size() ? 1.0 : 0.5));
      fillets[i] = choose_fillet(grid, points[i - 1], points[i], points[i + 1], available, config);
    }
    // Redistribute unused space in both directions; the neighbours' actual trims prevent overlap.
    for (int pass = 0; pass < 2; ++pass) {
      for (std::size_t j = 1; j + 1 < points.size(); ++j) {
        const std::size_t i = pass == 0 ? j : points.size() - 1 - j;
        const double available = std::max(
          0.0, std::min(
            distance(points[i - 1], points[i]) - fillets[i - 1].trim,
            distance(points[i], points[i + 1]) - fillets[i + 1].trim));
        const double old_radius = fillets[i].points.empty() ? 0.0 :
          1.0 / std::abs(fillets[i].points.front().curvature);
        fillets[i] = choose_fillet(
          grid, points[i - 1], points[i], points[i + 1], available, config, old_radius);
      }
    }
  }
  std::vector<PathPoint2D> rounded;
  const auto append = [&rounded, &config](PathPoint2D point) {
      if (!rounded.empty()) {
        const double scale = std::max(
          {std::abs(point.x), std::abs(point.y),
            std::abs(rounded.back().x), std::abs(rounded.back().y)});
        if (distance(xy(rounded.back()), xy(point)) <= roundoff(scale)) {
          rounded.back() = point;  // At touching fillets the next outgoing primitive wins.
          return;
        }
      }
      if (rounded.size() >= config.max_output_points) {
        throw std::length_error("Rounded path exceeds max_output_points");
      }
      rounded.push_back(point);
    };
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!fillets[i].points.empty()) {
      for (const auto & point : fillets[i].points) {
        append(point);
      }
      ++stats.rounded_corners;
    } else {
      const double yaw = i + 1 < points.size() ?
        std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x) :
        (rounded.empty() ? 0.0 : rounded.back().tangent_yaw);
      append({points[i].x, points[i].y, 0.0, yaw, 0.0, i > 0});
      if (i > 0 && i + 1 < points.size()) {
        ++stats.retained_corners;
        if (fillets[i].space_limited) {
          ++stats.space_limited_corners;
        } else {
          ++stats.collision_limited_corners;
        }
      }
    }
  }
  if (!rounded.empty()) {
    rounded.back().stop_required = true;
  }
  return rounded;
}

double path_cost(
  const std::vector<PathPoint2D> & points, const PathProcessingStats & stats,
  const PathProcessingConfig & config)
{
  double cost = config.retained_corner_cost * stats.retained_corners;
  for (std::size_t i = 1; i < points.size(); ++i) {
    cost += distance(xy(points[i - 1]), xy(points[i]));
  }
  return cost;
}
}  // namespace

void validate_path_processing_config(const PathProcessingConfig & config)
{
  if (!std::isfinite(config.resample_spacing) || config.resample_spacing <= 0.0 ||
    config.shortcut_max_lookahead == 0 || config.max_output_points == 0)
  {
    throw std::invalid_argument("Path spacing must be finite/positive and limits must be positive");
  }
  if (!std::isfinite(config.arc_radius) || !std::isfinite(config.arc_min_radius) ||
    !std::isfinite(config.arc_max_deviation) || !std::isfinite(config.arc_max_angle) ||
    config.arc_min_radius <= 0.0 || config.arc_radius < config.arc_min_radius ||
    config.arc_max_deviation <= 0.0 || config.arc_max_angle <= 0.0 ||
    config.arc_max_angle > std::acos(-1.0) / 2.0 ||
    config.arc_radius_samples < 2 || config.arc_radius_samples > 1000)
  {
    throw std::invalid_argument("Invalid arc radii, sampling limits or discretization bounds");
  }
  if (!std::isfinite(config.arc_radius_max) || config.arc_radius_max < 0.0 ||
    (config.arc_radius_max > 0.0 && config.arc_radius_max < config.arc_radius) ||
    !std::isfinite(config.arc_radius_weight) || config.arc_radius_weight < 0.0 ||
    !std::isfinite(config.arc_curvature_weight) || config.arc_curvature_weight < 0.0 ||
    !std::isfinite(config.retained_corner_cost) || config.retained_corner_cost < 0.0)
  {
    throw std::invalid_argument("Invalid radius-search costs or maximum radius");
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
  const PathProcessingConfig & config, PathProcessingStats * stats)
{
  validate_path_processing_config(config);
  PathProcessingStats processing_stats;
  if (stats != nullptr) {
    *stats = processing_stats;
  }
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
  const auto before_shortcut = simplified;
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
  auto rounded = round_corners(grid, simplified, config, processing_stats);
  if (config.enable_arc_smoothing && config.recover_shortcut_corners && config.enable_shortcut &&
    processing_stats.retained_corners > 0)
  {
    PathProcessingStats alternative_stats;
    try {
      auto alternative = round_corners(grid, before_shortcut, config, alternative_stats);
      if (path_cost(
          alternative, alternative_stats,
          config) < path_cost(rounded, processing_stats, config))
      {
        rounded = std::move(alternative);
        processing_stats = alternative_stats;
        processing_stats.shortcut_recovered = true;
      }
    } catch (const std::length_error &) {
      // Keep the already validated shortcut candidate if the alternative exceeds its work budget.
    }
  }
  double total_length = 0.0;
  for (std::size_t i = 1; i < rounded.size(); ++i) {
    total_length += std::hypot(
      rounded[i].x - rounded[i - 1].x, rounded[i].y - rounded[i - 1].y);
  }
  if (!std::isfinite(total_length) ||
    std::ceil(total_length / config.resample_spacing) + 1.0 > config.max_output_points)
  {
    throw std::length_error("Resampling would exceed max_output_points");
  }
  std::vector<PathPoint2D> output;
  const auto append = [&output, &config](PathPoint2D point) {
      if (!output.empty() && same_point(xy(output.back()), xy(point))) {
        return;
      }
      if (output.size() >= config.max_output_points) {
        throw std::length_error("Resampling including bends exceeds max_output_points");
      }
      output.push_back(point);
    };
  if (!rounded.empty()) {
    append(rounded.front());
  }
  double travelled = 0.0;
  std::size_t sample_index = 1;
  for (std::size_t i = 1; i < rounded.size(); ++i) {
    const auto a = rounded[i - 1], b = rounded[i];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    const double end = travelled + length;
    double next_sample = sample_index * config.resample_spacing;
    while (next_sample < end - roundoff(end)) {
      const double ratio = (next_sample - travelled) / length;
      const double end_yaw = b.stop_required ? std::atan2(b.y - a.y, b.x - a.x) : b.tangent_yaw;
      const double yaw_delta = std::remainder(end_yaw - a.tangent_yaw, 2.0 * std::acos(-1.0));
      append(
        {a.x + (b.x - a.x) * ratio, a.y + (b.y - a.y) * ratio, 0.0,
          a.tangent_yaw + ratio * yaw_delta, a.curvature, false});
      next_sample = ++sample_index * config.resample_spacing;
    }
    if (std::abs(next_sample - end) <= roundoff(end)) {
      ++sample_index;
    }
    // Retain bends and arc vertices: no new chord may bypass collision-checked geometry.
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
  if (stats != nullptr) {
    *stats = processing_stats;
  }
  return output;
}
}  // namespace plan

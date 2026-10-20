// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "path_optimizer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace plan
{
namespace
{
constexpr double kEpsilon = 1e-10;  // numerical equality, not a geometry safety margin
const double kPi = std::acos(-1.0);
Point2D xy(const PathPoint2D & p) {return {p.x, p.y};}
Point2D add(Point2D a, Point2D b) {return {a.x + b.x, a.y + b.y};}
Point2D sub(Point2D a, Point2D b) {return {a.x - b.x, a.y - b.y};}
Point2D mul(Point2D a, double s) {return {a.x * s, a.y * s};}
double dot(Point2D a, Point2D b) {return a.x * b.x + a.y * b.y;}
double cross(Point2D a, Point2D b) {return a.x * b.y - a.y * b.x;}
double norm(Point2D a) {return std::hypot(a.x, a.y);}
double dist(Point2D a, Point2D b) {return norm(sub(a, b));}
Point2D direction(double yaw) {return {std::cos(yaw), std::sin(yaw)};}
double heading(Point2D a, Point2D b) {return std::atan2(b.y - a.y, b.x - a.x);}
double wrap(double a) {return std::remainder(a, 2.0 * kPi);}

template<typename P>
double path_length(const std::vector<P> & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return length;
}

struct SearchExhausted {};
struct Budget
{
  std::size_t used = 0, maximum;
  bool exhausted = false;
  std::string reason;
  void spend()
  {
    if (used >= maximum) {
      exhausted = true;
      reason = "work_limit";
      throw SearchExhausted{};
    }
    ++used;
  }
};

struct Candidate
{
  std::vector<PathPoint2D> points;
  double trim_in = 0.0, trim_out = 0.0, length = 0.0, cost = 0.0;
  bool stop = false;
  std::uint8_t kind = 0;
};

struct Bend
{
  Point2D a, b, c, u, v;
  double yaw, turn, angle, sign;
  Bend(Point2D first, Point2D corner, Point2D last)
  : a(first), b(corner), c(last), u(mul(sub(b, a), 1.0 / dist(a, b))),
    v(mul(sub(c, b), 1.0 / dist(b, c))), yaw(heading(a, b)),
    turn(std::atan2(cross(u, v), dot(u, v))), angle(std::abs(turn)), sign(turn > 0 ? 1 : -1) {}
};

void append(std::vector<PathPoint2D> & path, PathPoint2D p, const PathProcessingConfig & cfg)
{
  if (!path.empty() && dist(xy(path.back()), xy(p)) <= kEpsilon) {
    p.stop_required = p.stop_required || path.back().stop_required;
    path.back() = p;
    return;
  }
  if (path.size() >= cfg.max_output_points) {
    throw std::length_error("Joint path exceeds max_output_points");
  }
  path.push_back(p);
}

PathPoint2D sample(
  Point2D p, double yaw, double k, std::uint8_t kind,
  double bound, double rate, const PathProcessingConfig & cfg)
{
  PathPoint2D result{p.x, p.y, yaw, yaw, k, false};
  result.segment_type = kind;
  result.curvature_bound = bound;
  result.curvature_rate_bound = rate;
  result.speed_limit = curve_limits(cfg.motion, bound, rate).speed;
  return result;
}

bool clothoid_candidate(
  const PlanningGrid & grid, const Bend & b, double radius, double transition,
  const PathProcessingConfig & cfg, Budget & budget, CornerDiagnostic & diag, Candidate & out)
{
  budget.spend();
  const double peak = std::min(1.0 / radius, b.angle / transition);
  const double rate = peak / transition;
  const auto limits = curve_limits(cfg.motion, peak, rate);
  if (std::min(cfg.motion.v_nominal, limits.speed) < cfg.motion.min_turn_speed) {
    ++diag.speed;
    return false;
  }
  const double middle = std::max(0.0, b.angle / peak - transition);
  const double total = 2.0 * transition + middle;
  const auto yaw_at = [&](double s) {
      if (s <= transition) {
        return b.sign * peak * s * s / (2 * transition);
      }
      if (s <= transition + middle) {
        return b.sign * peak * (s - transition / 2);
      }
      return b.turn - b.sign * peak * std::pow(std::max(0.0, total - s), 2) / (2 * transition);
    };
  const auto k_at = [&](double s) {
      return b.sign * peak * std::clamp(std::min(s, total - s) / transition, 0.0, 1.0);
    };
  const double step = std::min(
    {cfg.resample_spacing, cfg.arc_max_angle / peak,
      std::sqrt(4 * cfg.arc_max_deviation / peak)});
  std::vector<double> ss{0.0};
  std::vector<Point2D> local{{0.0, 0.0}};
  double integration_error = 0.0;
  const double third = rate + peak * peak;
  for (auto stage : std::vector<std::pair<double, double>>{
        {0.0, transition}, {transition, middle}, {transition + middle, transition}})
  {
    const double count = std::ceil(stage.second / step);
    if (!std::isfinite(count) || count + ss.size() > cfg.max_output_points) {
      throw SearchExhausted{};
    }
    for (std::size_t j = 1; j <= static_cast<std::size_t>(count); ++j) {
      budget.spend();
      const double s = stage.first + stage.second * j / count;
      const double h = s - ss.back();
      const double subdivisions = std::max(
        1.0, std::ceil(
          std::sqrt(
            total * third * h * h * 16.0 / (24.0 * cfg.arc_max_deviation))));
      if (!std::isfinite(subdivisions) || subdivisions > cfg.joint_work_budget) {
        throw SearchExhausted{};
      }
      const double ds = h / subdivisions;
      auto p = local.back();
      for (std::size_t q = 0; q < static_cast<std::size_t>(subdivisions); ++q) {
        budget.spend();
        p = add(p, mul(direction(yaw_at(ss.back() + (q + .5) * ds)), ds));
      }
      integration_error += third * h * ds * ds / 24.0;
      local.push_back(p);
      ss.push_back(s);
    }
  }
  const double half = b.turn / 2.0;
  const double trim = dot(local.back(), direction(half)) / (2.0 * std::cos(half));
  if (!std::isfinite(trim) || trim > std::min(dist(b.a, b.b), dist(b.b, b.c))) {
    ++diag.space;
    return false;
  }
  const auto entry = sub(b.b, mul(b.u, trim)), exit = add(b.b, mul(b.v, trim));
  const auto world = [&](Point2D p) {
      return add(entry, {b.u.x * p.x - b.u.y * p.y, b.u.y * p.x + b.u.x * p.y});
    };
  const double error = integration_error + dist(world(local.back()), exit) + kEpsilon;
  for (std::size_t j = 0; j < ss.size(); ++j) {
    const auto p = j + 1 == ss.size() ? exit : world(local[j]);
    if (j > 0) {
      const double h = ss[j] - ss[j - 1], chord = dist(xy(out.points.back()), p);
      if (!curve_chord_is_free(grid, xy(out.points.back()), p, error + peak * h * h / 8)) {
        ++diag.collision;
        return false;
      }
      const double mid = (ss[j] + ss[j - 1]) / 2;
      auto & prev = out.points.back();
      prev.curvature_bound = std::max(std::abs(k_at(ss[j])), std::abs(k_at(ss[j - 1])));
      prev.curvature_rate_bound = (mid<transition || mid> transition + middle) ?
        rate * h / chord : 0.0;
      prev.speed_limit = curve_limits(
        cfg.motion, prev.curvature_bound,
        prev.curvature_rate_bound).speed;
    }
    out.points.push_back(sample(p, b.yaw + yaw_at(ss[j]), k_at(ss[j]), 1, peak, rate, cfg));
  }
  out.trim_in = out.trim_out = trim;
  out.kind = 1;
  return true;
}

Point2D closest_line(Point2D p, Point2D a, Point2D b)
{
  const auto d = sub(b, a);
  const double q = dot(d, d);
  return add(a, mul(d, q > 0.0 ? std::clamp(dot(sub(p, a), d) / q, 0.0, 1.0) : 0.0));
}

struct Circle
{
  const Bend & bend;
  double radius, trim, length, start_angle;
  Point2D entry, exit, centre;
  Circle(const Bend & b, double r)
  : bend(b), radius(r), trim(r * std::tan(b.angle / 2)), length(r * b.angle),
    start_angle(b.yaw - b.sign * kPi / 2), entry(sub(b.b, mul(b.u, trim))),
    exit(add(b.b, mul(b.v, trim))), centre(add(entry, {-b.sign * r * b.u.y, b.sign * r * b.u.x})) {}
  Point2D at(double s) const
  {
    return add(centre, mul(direction(start_angle + bend.sign * s / radius), radius));
  }
  Point2D closest(Point2D p) const
  {
    auto best = closest_line(p, bend.a, entry);
    const auto consider = [&](Point2D q) {
        if (dist(p, q) < dist(p, best)) {best = q;}
      };
    consider(closest_line(p, exit, bend.c));
    consider(entry);
    consider(exit);
    const double angle = bend.sign * wrap(heading(centre, p) - start_angle);
    if (angle >= 0.0 && angle <= bend.angle) {consider(at(angle * radius));}
    return best;
  }
};

using Bezier = std::array<Point2D, 6>;
Bezier hermite(Point2D a, double yaw_a, double ka, Point2D b, double yaw_b, double kb, double h)
{
  const auto da = mul(direction(yaw_a), h), db = mul(direction(yaw_b), h);
  const auto dda = mul(direction(yaw_a + kPi / 2), h * h * ka);
  const auto ddb = mul(direction(yaw_b + kPi / 2), h * h * kb);
  return {a, add(a, mul(da, .2)), add(add(a, mul(da, .4)), mul(dda, .05)),
    add(sub(b, mul(db, .4)), mul(ddb, .05)), sub(b, mul(db, .2)), b};
}

std::pair<Bezier, Bezier> split(const Bezier & b)
{
  Bezier left{}, right{}, work = b;
  left[0] = b[0]; right[5] = b[5];
  for (std::size_t j = 1; j < 6; ++j) {
    for (std::size_t i = 0; i < 6 - j; ++i) {
      work[i] = mul(add(work[i], work[i + 1]), .5);
    }
    left[j] = work[0]; right[5 - j] = work[5 - j];
  }
  return {left, right};
}

double choose(unsigned n, unsigned k)
{
  double r = 1.0;
  for (unsigned i = 1; i <= k; ++i) {
    r *= static_cast<double>(n + 1 - i) / i;
  }
  return r;
}

// Bernstein coefficients bound products on the ENTIRE interval, not only samples.
double product_bound(const std::vector<Point2D> & a, const std::vector<Point2D> & b, bool use_dot)
{
  const auto n = static_cast<unsigned>(a.size() - 1), m = static_cast<unsigned>(b.size() - 1);
  double bound = 0.0;
  for (unsigned k = 0; k <= n + m; ++k) {
    double coefficient = 0.0;
    for (unsigned i = 0; i <= n; ++i) {
      if (k < i || k - i > m) {continue;}
      coefficient += choose(n, i) * choose(m, k - i) / choose(n + m, k) *
        (use_dot ? dot(a[i], b[k - i]) : cross(a[i], b[k - i]));
    }
    bound = std::max(bound, std::abs(coefficient));
  }
  return bound;
}

std::vector<Point2D> derivative(const std::vector<Point2D> & p)
{
  std::vector<Point2D> d;
  for (std::size_t i = 1; i < p.size(); ++i) {
    d.push_back(mul(sub(p[i], p[i - 1]), p.size() - 1));
  }
  return d;
}

bool bezier_samples(
  const PlanningGrid & grid, const Circle & circle, const Bezier & b,
  std::size_t depth, const PathProcessingConfig & cfg, Budget & budget,
  CornerDiagnostic & diag, std::vector<PathPoint2D> & out)
{
  budget.spend();
  const auto halves = split(b);
  const auto mid = halves.first.back();
  if (dist(mid, circle.closest(mid)) > cfg.fallback_deviation) {++diag.deviation; return false;}
  const auto d1 = derivative(std::vector<Point2D>(b.begin(), b.end()));
  const auto d2 = derivative(d1), d3 = derivative(d2);
  const auto chord = sub(b.back(), b.front());
  const double chord_length = norm(chord);
  double lower = std::numeric_limits<double>::infinity(), envelope = 0.0, deviation = 0.0;
  const auto ref = circle.closest(mid);
  for (const auto & p : d1) {
    lower = std::min(lower, chord_length > 0 ? dot(p, chord) / chord_length : 0.0);
  }
  for (const auto & p : b) {
    envelope = std::max(envelope, dist(p, closest_line(p, b.front(), b.back())));
    deviation = std::max(deviation, dist(p, ref));
  }
  const double cross12 = product_bound(d1, d2, false);
  const double kb = lower > 0 ? cross12 / std::pow(lower, 3) : INFINITY;
  const double rate = lower > 0 ? product_bound(d1, d3, false) / std::pow(lower, 4) +
    3 * cross12 * product_bound(d1, d2, true) / std::pow(lower, 6) : INFINITY;
  const auto curvature = [&](bool end) {
      const auto p = end ? d1.back() : d1.front(), q = end ? d2.back() : d2.front();
      return cross(p, q) / std::pow(norm(p), 3);
    };
  if (!std::isfinite(curvature(false)) || !std::isfinite(curvature(true)) ||
    std::max(std::abs(curvature(false)), std::abs(curvature(true))) >
    1.0 / cfg.arc_min_radius + kEpsilon)
  {
    ++diag.curvature; return false;
  }
  const bool regular = lower > kEpsilon && kb <= 1.0 / cfg.arc_min_radius + kEpsilon;
  const bool speed_ok = regular && std::isfinite(rate) &&
    curve_limits(cfg.motion, kb, rate).speed >= cfg.motion.min_turn_speed;
  const bool small = chord_length <= cfg.resample_spacing && envelope <= cfg.arc_max_deviation &&
    std::abs(wrap(heading({0, 0}, d1.back()) - heading({0, 0}, d1.front()))) <= cfg.arc_max_angle;
  if (!regular || !speed_ok || !small || deviation > cfg.fallback_deviation) {
    if (depth >= cfg.curve_max_depth) {
      if (!regular) {++diag.curvature;} else if (!speed_ok) {++diag.speed;} else {++diag.deviation;}
      return false;
    }
    return bezier_samples(grid, circle, halves.first, depth + 1, cfg, budget, diag, out) &&
           bezier_samples(grid, circle, halves.second, depth + 1, cfg, budget, diag, out);
  }
  if (!curve_chord_is_free(grid, b.front(), b.back(), envelope + kEpsilon)) {
    ++diag.collision; return false;
  }
  append(
    out, sample(
      b.front(), heading({0, 0}, d1.front()), curvature(
        false), 2, kb, rate, cfg), cfg);
  append(out, sample(b.back(), heading({0, 0}, d1.back()), curvature(true), 2, 0, 0, cfg), cfg);
  return true;
}

bool circular_samples(
  const PlanningGrid & grid, const Circle & c, double from, double to,
  double padding, std::uint8_t kind, const PathProcessingConfig & cfg, Budget & budget,
  std::vector<PathPoint2D> & out)
{
  const double step = std::min(
    {cfg.resample_spacing, cfg.arc_max_angle * c.radius,
      std::sqrt(4 * cfg.arc_max_deviation * c.radius)});
  const double count = std::max(1.0, std::ceil((to - from) / step));
  if (!std::isfinite(count) || count + out.size() > cfg.max_output_points) {
    throw SearchExhausted{};
  }
  const double ds = (to - from) / count;
  for (std::size_t j = 0; j <= static_cast<std::size_t>(count); ++j) {
    budget.spend();
    const double s = from + j * ds;
    const auto p = c.at(s);
    if (j > 0 && !curve_chord_is_free(
        grid, c.at(s - ds), p,
        padding + ds * ds / (8 * c.radius) + kEpsilon)) {return false;}
    if (padding == 0.0) {
      append(
        out, sample(
          p, c.bend.yaw + c.bend.sign * s / c.radius,
          c.bend.sign / c.radius, kind, 1 / c.radius, 0, cfg), cfg);
    }
  }
  return true;
}

bool fallback_candidate(
  const PlanningGrid & grid, const Bend & b, double radius, double blend,
  const PathProcessingConfig & cfg, Budget & budget, CornerDiagnostic & diag, Candidate & out)
{
  budget.spend();
  const Circle c(b, radius);
  const double trim = c.trim + blend;
  if (trim > std::min(dist(b.a, b.b), dist(b.b, b.c)) || 2 * blend > c.length) {
    ++diag.space; return false;
  }
  if (!curve_chord_is_free(grid, sub(c.entry, mul(b.u, blend)), c.entry, cfg.fallback_deviation) ||
    !curve_chord_is_free(grid, c.exit, add(c.exit, mul(b.v, blend)), cfg.fallback_deviation) ||
    !circular_samples(grid, c, 0, c.length, cfg.fallback_deviation, 2, cfg, budget, out.points))
  {++diag.collision; return false;}
  const auto first = hermite(
    sub(c.entry, mul(b.u, blend)), b.yaw, 0.0, c.at(blend),
    b.yaw + b.sign * blend / radius, b.sign / radius, 2 * blend);
  const auto last = hermite(
    c.at(c.length - blend), b.yaw + b.turn - b.sign * blend / radius,
    b.sign / radius, add(c.exit, mul(b.v, blend)), b.yaw + b.turn, 0.0, 2 * blend);
  if (!bezier_samples(grid, c, first, 0, cfg, budget, diag, out.points) ||
    !circular_samples(grid, c, blend, c.length - blend, 0, 2, cfg, budget, out.points) ||
    !bezier_samples(grid, c, last, 0, cfg, budget, diag, out.points)) {return false;}
  out.trim_in = out.trim_out = trim;
  out.kind = 2;
  return true;
}

// Every cost term is in seconds (integrals normalized by curve length).
double geometry_cost(
  const PlanningGrid & grid, const std::vector<PathPoint2D> & p,
  const PathProcessingConfig & cfg)
{
  double time = 0, smooth = 0, wall = 0, length = 0;
  for (std::size_t i = 1; i < p.size(); ++i) {
    const double ds = dist(xy(p[i - 1]), xy(p[i]));
    const auto & q = p[i - 1];
    const double speed = std::min(cfg.motion.v_nominal, q.speed_limit);
    if (speed <= 0) {return INFINITY;}
    time += ds / speed;
    smooth += ds * (std::pow(q.curvature_bound * cfg.arc_min_radius, 2) +
      std::pow(std::max(0.0, q.curvature_rate_bound / cfg.max_curvature_rate - 1.0), 2));
    const auto mid = mul(add(xy(q), xy(p[i])), .5);
    const double d = grid.clearance_at(mid.x, mid.y);
    wall += ds * std::pow(std::max(0.0, 1.0 - d / cfg.clearance_target), 2);
    length += ds;
  }
  return time + (length > 0 ? (cfg.smoothness_weight * smooth + cfg.clearance_weight * wall) /
         length : 0.0);
}

std::vector<Candidate> candidates(
  const PlanningGrid & grid, const Bend & b,
  const PathProcessingConfig & cfg, Budget & budget, CornerDiagnostic & diag)
{
  Candidate stop;
  stop.stop = true;
  auto point = sample(b.b, b.yaw + b.turn, 0, 0, 0, 0, cfg);
  point.stop_required = true;
  stop.points.push_back(point);
  stop.cost = b.angle / cfg.motion.w_max;
  std::vector<Candidate> result{stop};
  if (b.angle <= kEpsilon || b.angle >= kPi - kEpsilon) {++diag.space; return result;}
  const double maximum = cfg.arc_radius_max > 0 ? cfg.arc_radius_max : cfg.arc_radius;
  std::vector<double> radii{cfg.arc_radius};
  for (std::size_t i = 0; i < cfg.arc_radius_samples; ++i) {
    const double ratio = static_cast<double>(i) / (cfg.arc_radius_samples - 1);
    radii.push_back(cfg.arc_min_radius + ratio * (maximum - cfg.arc_min_radius));
  }
  const auto retain = [&](Candidate c, double radius, double transition, double blend) {
      double minimum_speed = cfg.motion.v_max;
      for (std::size_t i = 0; i + 1 < c.points.size(); ++i) {
        minimum_speed = std::min(minimum_speed, c.points[i].speed_limit);
      }
      if (minimum_speed < cfg.motion.min_turn_speed) {
        ++diag.speed;
        diag.trials.push_back(
          {c.kind, radius, transition, blend, c.trim_in, c.trim_out,
            minimum_speed, "minimum_speed"});
        return;
      }
      c.length = path_length(c.points);
      // Length and radius/curvature costs are normalized before comparison. Avoid
      // a huge equivalent-radius penalty for intentionally gentle triangular turns.
      c.cost = geometry_cost(grid, c.points, cfg) + cfg.arc_radius_weight *
        std::pow(
        std::min(
          maximum, 1.0 / std::max(
            kEpsilon,
            std::abs(c.points[c.points.size() / 2].curvature))) - cfg.arc_radius, 2);
      if (result.size() >= cfg.joint_max_candidates) {
        budget.reason = "candidate_limit";
        throw SearchExhausted{};
      }
      diag.trials.push_back(
        {c.kind, radius, transition, blend, c.trim_in, c.trim_out,
          minimum_speed, "feasible"});
      result.push_back(std::move(c));
    };
  const auto rejected = [&](const CornerDiagnostic & before, std::uint8_t kind,
      double radius, double transition, double blend) {
      const std::string reason = diag.space > before.space ? "space" :
        diag.collision > before.collision ? "collision" :
        diag.curvature > before.curvature ? "curvature" :
        diag.speed > before.speed ? "minimum_speed" :
        diag.deviation > before.deviation ? "deviation" : "geometry";
      diag.trials.push_back({kind, radius, transition, blend, 0, 0, 0, reason});
    };
  for (double radius : radii) {
    for (std::size_t j = 0; j < cfg.transition_samples; ++j) {
      const double ratio = static_cast<double>(j) / (cfg.transition_samples - 1);
      const double length = cfg.transition_min_length + ratio *
        (cfg.transition_length - cfg.transition_min_length);
      Candidate candidate;
      ++diag.candidates;
      const auto before = diag;
      if (clothoid_candidate(grid, b, radius, length, cfg, budget, diag, candidate)) {
        retain(std::move(candidate), radius, length, 0);
      } else {rejected(before, 1, radius, length, 0);}
    }
  }
  if (cfg.enable_arc_fallback) {
    for (double radius : radii) {
      for (std::size_t j = 0; j < cfg.fallback_blend_samples; ++j) {
        const double ratio = static_cast<double>(j) / (cfg.fallback_blend_samples - 1);
        Candidate candidate;
        ++diag.candidates;
        const auto before = diag;
        const double blend = cfg.fallback_blend_min + ratio *
          (cfg.fallback_blend_max - cfg.fallback_blend_min);
        if (fallback_candidate(grid, b, radius, blend, cfg, budget, diag, candidate)) {
          retain(std::move(candidate), radius, 0, blend);
        } else {rejected(before, 2, radius, 0, blend);}
      }
    }
  }
  return result;
}

void line_samples(
  std::vector<PathPoint2D> & out, Point2D a, Point2D b,
  const PathProcessingConfig & cfg)
{
  const double length = dist(a, b);
  if (length <= kEpsilon) {return;}
  const double yaw = heading(a, b);
  const auto count = static_cast<std::size_t>(std::ceil(length / cfg.resample_spacing));
  for (std::size_t i = 0; i <= count; ++i) {
    const auto p = add(a, mul(sub(b, a), static_cast<double>(i) / count));
    append(out, sample(p, yaw, 0, 0, 0, 0, cfg), cfg);
  }
}

// Forward and backward passes give an executable envelope, including true stops.
double traversal_time(std::vector<PathPoint2D> & p, const PathProcessingConfig & cfg)
{
  if (p.size() < 2) {return 0.0;}
  std::vector<double> speed(p.size(), cfg.motion.v_nominal);
  std::vector<CurveLimits> limits(p.size());
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    limits[i] = curve_limits(cfg.motion, p[i].curvature_bound, p[i].curvature_rate_bound);
    const double cap = std::min(limits[i].speed, p[i].speed_limit);
    speed[i] = std::min(speed[i], cap);
    speed[i + 1] = std::min(speed[i + 1], cap);
  }
  speed.front() = 0;
  for (std::size_t i = 0; i < p.size(); ++i) {
    if (p[i].stop_required) {
      speed[i] = 0;
    }
  }
  for (std::size_t i = p.size() - 1; i > 0; --i) {
    const double ds = dist(xy(p[i - 1]), xy(p[i]));
    speed[i - 1] = std::min(
      speed[i - 1], std::sqrt(
        speed[i] * speed[i] +
        2 * limits[i - 1].deceleration * ds));
  }
  for (std::size_t i = 1; i < p.size(); ++i) {
    const double ds = dist(xy(p[i - 1]), xy(p[i]));
    speed[i] = std::min(
      speed[i], std::sqrt(
        speed[i - 1] * speed[i - 1] +
        2 * limits[i - 1].acceleration * ds));
  }
  double time = 0;
  for (std::size_t i = 1; i < p.size(); ++i) {
    const double ds = dist(xy(p[i - 1]), xy(p[i]));
    // Integrate a trapezoidal/triangular velocity profile; two zero endpoints do
    // not imply an impossible straight segment between two mandatory stops.
    const double a = limits[i - 1].acceleration, d = limits[i - 1].deceleration;
    const double peak = std::min(
      {cfg.motion.v_nominal, p[i - 1].speed_limit,
        std::sqrt(
          (2 * ds + speed[i - 1] * speed[i - 1] / a + speed[i] * speed[i] / d) /
          (1.0 / a + 1.0 / d))});
    if (peak <= 0.0) {return INFINITY;}
    const double accelerate = std::max(0.0, (peak * peak - speed[i - 1] * speed[i - 1]) / (2 * a));
    const double decelerate = std::max(0.0, (peak * peak - speed[i] * speed[i]) / (2 * d));
    time += std::max(0.0, peak - speed[i - 1]) / a + std::max(0.0, peak - speed[i]) / d +
      std::max(0.0, ds - accelerate - decelerate) / peak;
    if (p[i].stop_required && i + 1 < p.size()) {
      time += std::abs(wrap(p[i].tangent_yaw - heading(xy(p[i - 1]), xy(p[i])))) / cfg.motion.w_max;
    }
  }
  return time;
}

struct Score
{
  std::size_t stops = 0, fallbacks = 0;
  double cost = 0, length = 0;
};
bool better(const Score & a, const Score & b)
{
  return std::tie(a.stops, a.fallbacks, a.cost, a.length) <
         std::tie(b.stops, b.fallbacks, b.cost, b.length);
}

Score evaluate(
  const PlanningGrid & grid, std::vector<PathPoint2D> & p,
  const PathProcessingConfig & cfg)
{
  Score result;
  result.length = path_length(p);
  bool in_fallback = false;
  for (std::size_t i = 0; i < p.size(); ++i) {
    result.stops += p[i].stop_required && i + 1 < p.size();
    const bool fallback = p[i].segment_type == 2;
    result.fallbacks += fallback && !in_fallback;
    in_fallback = fallback;
    if (i + 1 < p.size()) {p[i].yaw = heading(xy(p[i]), xy(p[i + 1]));} else if (i) {
      p[i].yaw = p[i - 1].yaw;
    }
  }
  result.cost = traversal_time(p, cfg);
  // geometry_cost contains an approximate traversal time; retain only the normalized penalties.
  double nominal_time = 0;
  for (std::size_t i = 1; i < p.size(); ++i) {
    nominal_time += dist(xy(p[i - 1]), xy(p[i])) /
      std::min(cfg.motion.v_nominal, p[i - 1].speed_limit);
  }
  result.cost += geometry_cost(grid, p, cfg) - nominal_time;
  return result;
}

struct Solution
{
  std::vector<PathPoint2D> points;
  std::vector<CornerDiagnostic> diagnostics;
  Score score;
};

struct Label
{
  Score score;
  std::size_t previous = 0, label = 0;
};

bool dominates(const Score & a, const Score & b)
{
  const auto qa = std::make_pair(a.stops, a.fallbacks), qb = std::make_pair(b.stops, b.fallbacks);
  return a.length <= b.length && (qa < qb || (qa == qb && a.cost <= b.cost));
}

Solution solve(
  const PlanningGrid & grid, const std::vector<Point2D> & vertices,
  double length_budget, const PathProcessingConfig & cfg, Budget & budget)
{
  std::vector<std::vector<Candidate>> sets(vertices.size());
  std::vector<CornerDiagnostic> diagnostics(vertices.size());
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    if (i == 0 || i + 1 == vertices.size()) {
      Candidate endpoint;
      endpoint.points.push_back(
        sample(
          vertices[i], i == 0 ? heading(vertices[0], vertices[1]) :
          heading(vertices[i - 1], vertices[i]), 0, 0, 0, 0, cfg));
      sets[i].push_back(endpoint);
    } else {
      diagnostics[i].x = vertices[i].x;
      diagnostics[i].y = vertices[i].y;
      sets[i] = candidates(
        grid, Bend(vertices[i - 1], vertices[i], vertices[i + 1]),
        cfg, budget, diagnostics[i]);
    }
  }
  using Layer = std::vector<std::vector<Label>>;
  std::vector<Layer> layers(vertices.size());
  layers[0].resize(1);
  layers[0][0].push_back({});
  for (std::size_t i = 1; i < vertices.size(); ++i) {
    layers[i].resize(sets[i].size());
    for (std::size_t j = 0; j < sets[i].size(); ++j) {
      const auto & current = sets[i][j];
      for (std::size_t k = 0; k < sets[i - 1].size(); ++k) {
        budget.spend();
        const auto & previous = sets[i - 1][k];
        const double connector = dist(vertices[i - 1], vertices[i]) -
          previous.trim_out - current.trim_in;
        if (connector < -kEpsilon) {++diagnostics[i].conflicts; continue;}
        if (!line_of_sight(grid, xy(previous.points.back()), xy(current.points.front()))) {
          ++diagnostics[i].collision; continue;
        }
        for (std::size_t q = 0; q < layers[i - 1][k].size(); ++q) {
          budget.spend();
          auto score = layers[i - 1][k][q].score;
          score.length += std::max(0.0, connector) + current.length;
          score.cost += std::max(0.0, connector) / cfg.motion.v_nominal + current.cost;
          score.stops += current.stop;
          score.fallbacks += current.kind == 2;
          if (score.length + dist(xy(current.points.back()), vertices.back()) >
            length_budget + kEpsilon)
          {++diagnostics[i].length; continue;}
          auto & labels = layers[i][j];
          if (std::any_of(
              labels.begin(), labels.end(), [&](const Label & l) {
                return dominates(l.score, score);
              })) {continue;}
          labels.erase(
            std::remove_if(
              labels.begin(), labels.end(), [&](const Label & l) {
                return dominates(score, l.score);
              }), labels.end());
          labels.push_back({score, k, q});
          if (labels.size() > cfg.joint_max_labels) {
            // Bound the Pareto frontier without abandoning other route variants.
            // Preserve the shortest label as well as the best lexicographic labels.
            budget.exhausted = true;
            if (budget.reason.empty()) {budget.reason = "label_limit";}
            const auto shortest = *std::min_element(
              labels.begin(), labels.end(),
              [](const Label & a, const Label & b) {return a.score.length < b.score.length;});
            std::stable_sort(
              labels.begin(), labels.end(), [](const Label & a, const Label & b) {
                return better(a.score, b.score);
              });
            labels.resize(cfg.joint_max_labels);
            if (shortest.score.length < labels.back().score.length) {labels.back() = shortest;}
          }
        }
      }
    }
  }
  Solution best;
  best.score.cost = INFINITY;
  best.score.stops = std::numeric_limits<std::size_t>::max();
  for (std::size_t q = 0; q < layers.back()[0].size(); ++q) {
    std::vector<std::size_t> selected(vertices.size());
    std::size_t k = 0, label = q;
    for (std::size_t i = vertices.size() - 1; i > 0; --i) {
      selected[i] = k;
      const auto & l = layers[i][k][label];
      k = l.previous; label = l.label;
    }
    std::vector<PathPoint2D> points;
    append(points, sets[0][0].points[0], cfg);
    auto report = diagnostics;
    for (std::size_t i = 1; i < vertices.size(); ++i) {
      const auto & c = sets[i][selected[i]];
      line_samples(points, xy(points.back()), xy(c.points.front()), cfg);
      for (const auto & point : c.points) {
        append(points, point, cfg);
      }
      report[i].retained = c.stop;
      report[i].chosen = selected[i];
    }
    points.back().stop_required = true;
    points.back().speed_limit = 0;
    auto score = evaluate(grid, points, cfg);
    // Touching fallback corners can have no intervening line sample. Count the
    // selected corner primitives, rather than connected runs of segment types.
    score.fallbacks = layers.back()[0][q].score.fallbacks;
    if (score.length <= length_budget + kEpsilon && std::isfinite(score.cost) &&
      better(score, best.score)) {best = {std::move(points), std::move(report), score};}
  }
  if (best.points.empty()) {best.diagnostics = diagnostics;}
  return best;
}

std::vector<Point2D> simplify(const std::vector<Point2D> & raw)
{
  std::vector<Point2D> result;
  for (const auto p : raw) {
    if (!result.empty() && dist(result.back(), p) <= kEpsilon) {continue;}
    while (result.size() >= 2) {
      const auto a = sub(result.back(), result[result.size() - 2]), b = sub(p, result.back());
      if (std::abs(cross(a, b)) > kEpsilon * norm(a) * norm(b) || dot(a, b) <= 0) {break;}
      result.pop_back();
    }
    result.push_back(p);
  }
  return result;
}

std::vector<Point2D> shortcut(
  const PlanningGrid & grid, const std::vector<Point2D> & p,
  const PathProcessingConfig & cfg, Budget & budget)
{
  std::vector<Point2D> out{p.front()};
  for (std::size_t i = 0; i + 1 < p.size(); ) {
    auto j = i + std::min(cfg.shortcut_max_lookahead, p.size() - 1 - i);
    while (j > i + 1) {
      budget.spend();
      if (line_of_sight(grid, p[i], p[j])) {break;}
      --j;
    }
    out.push_back(p[j]); i = j;
  }
  return out;
}

Point2D at(const std::vector<Point2D> & p, const std::vector<double> & s, double q)
{
  const auto upper = std::upper_bound(s.begin(), s.end(), q);
  const auto i = upper == s.begin() ? 0U : std::min(
    static_cast<std::size_t>(upper - s.begin() - 1), p.size() - 2);
  return add(p[i], mul(sub(p[i + 1], p[i]), std::clamp((q - s[i]) / (s[i + 1] - s[i]), 0.0, 1.0)));
}

double smoothstep(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * t * (10 + t * (-15 + 6 * t));
}

std::vector<Point2D> relocate(
  const PlanningGrid & grid, const std::vector<Point2D> & p,
  std::size_t corner, double shift, const PathProcessingConfig & cfg, Budget & budget)
{
  std::vector<double> ss(p.size(), 0.0);
  for (std::size_t i = 1; i < p.size(); ++i) {
    ss[i] = ss[i - 1] + dist(p[i - 1], p[i]);
  }
  const double centre = ss[corner], begin = std::max(0.0, centre - cfg.local_window_length);
  const double end = std::min(ss.back(), centre + cfg.local_window_length);
  const auto from = at(p, ss, begin), to = at(p, ss, end);
  const auto tangent = sub(to, from);
  if (norm(tangent) <= kEpsilon) {return {};}
  const Point2D normal{-tangent.y / norm(tangent), tangent.x / norm(tangent)};
  std::vector<double> knots = ss;
  // Move corridor anchors, then let the same curve optimizer join them. Dense
  // samples of the displacement bump would introduce many artificial bends.
  for (double edge : {begin, end}) {
    if (std::all_of(
        ss.begin(), ss.end(), [&](double s) {
          return std::abs(s - edge) >= cfg.local_anchor_spacing;
        })) {knots.push_back(edge);}
  }
  std::sort(knots.begin(), knots.end());
  knots.erase(
    std::unique(
      knots.begin(), knots.end(), [](double a, double b) {
        return std::abs(a - b) < kEpsilon;
      }), knots.end());
  std::vector<Point2D> out;
  for (const double s : knots) {
    budget.spend();
    auto point = at(p, ss, s);
    if (s > begin && s < end) {
      const double envelope = smoothstep((s - begin) / (centre - begin)) *
        smoothstep((end - s) / (end - centre));
      point = add(point, mul(normal, shift * envelope));
    }
    if (!out.empty() && !line_of_sight(grid, out.back(), point)) {return {};}
    out.push_back(point);
  }
  out.front() = p.front(); out.back() = p.back();
  return simplify(out);
}

void decorate_legacy(std::vector<PathPoint2D> & p, const PathProcessingConfig & cfg)
{
  for (std::size_t i = 0; i < p.size(); ++i) {
    double bound = std::abs(p[i].curvature), rate = 0;
    if (i + 1 < p.size() && !p[i + 1].stop_required) {
      bound = std::max(bound, std::abs(p[i + 1].curvature));
      rate = std::abs(p[i + 1].curvature - p[i].curvature) / dist(xy(p[i]), xy(p[i + 1]));
    }
    p[i].curvature_bound = bound; p[i].curvature_rate_bound = rate;
    p[i].segment_type = bound > 0 || rate > 0 ? 1 : 0;
    p[i].speed_limit = curve_limits(cfg.motion, bound, rate).speed;
  }
  if (!p.empty()) {p.back().speed_limit = 0;}
}
}  // namespace

std::vector<PathPoint2D> optimize_joint_path(
  const PlanningGrid & grid,
  const std::vector<Point2D> & raw, const std::vector<PathPoint2D> & baseline,
  const PathProcessingConfig & cfg, PathProcessingStats & stats)
{
  if (baseline.empty()) {return baseline;}
  auto best = baseline;
  decorate_legacy(best, cfg);
  if (std::any_of(
      best.begin(), best.end() - 1, [&](const PathPoint2D & p) {
        return p.speed_limit < cfg.motion.min_turn_speed;
      }))
  {
    // A geometry-only legacy incumbent may be infeasible under tightened shared
    // limits. Its already checked chords remain a safe stop/align fallback.
    for (std::size_t i = 0; i < best.size(); ++i) {
      const double yaw = i + 1 < best.size() ? heading(xy(best[i]), xy(best[i + 1])) :
        best[i - 1].tangent_yaw;
      const auto p = xy(best[i]);
      best[i] = sample(p, yaw, 0, 0, 0, 0, cfg);
      best[i].stop_required = i > 0;
    }
    best.back().speed_limit = 0;
  }
  auto best_score = evaluate(grid, best, cfg);
  stats.baseline_length = path_length(baseline);
  const double maximum_length = stats.baseline_length * cfg.max_length_ratio;
  Budget budget{0, cfg.joint_work_budget};
  std::size_t variants = 0;
  bool selected_joint = false, relocated = false;
  std::vector<CornerDiagnostic> selected_diagnostics;
  struct Variant
  {
    std::vector<Point2D> vertices;
    Score score;
    bool moved;
    std::vector<CornerDiagnostic> diagnostics;
  };
  std::vector<Variant> beam;
  const auto consider = [&](std::vector<Point2D> vertices, bool moved) {
      ++variants;
      auto solution = solve(grid, vertices, maximum_length, cfg, budget);
      if (selected_diagnostics.empty()) {selected_diagnostics = solution.diagnostics;}
      if (std::isfinite(solution.score.cost)) {
        beam.push_back({vertices, solution.score, moved, solution.diagnostics});
        if (better(solution.score, best_score)) {
          best = std::move(solution.points); best_score = solution.score;
          selected_diagnostics = std::move(solution.diagnostics);
          selected_joint = true; relocated = moved;
        }
      }
    };
  try {
    auto vertices = simplify(raw);
    if (vertices.size() >= 3) {
      consider(vertices, false);
      if (cfg.enable_shortcut) {consider(shortcut(grid, vertices, cfg, budget), false);}
      // Two bounded beam rounds permit coordinated changes around neighbouring bends.
      for (int pass = 0; cfg.enable_local_relocation && pass < 2; ++pass) {
        std::stable_sort(
          beam.begin(), beam.end(), [](const Variant & a, const Variant & b) {
            return better(a.score, b.score);
          });
        if (beam.size() > cfg.local_beam_width) {beam.resize(cfg.local_beam_width);}
        const auto seeds = beam;
        for (const auto & seed : seeds) {
          std::vector<std::size_t> corners;
          for (std::size_t i = 1; i + 1 < seed.vertices.size(); ++i) {
            corners.push_back(i);
          }
          const auto priority = [&](std::size_t i) {
              if (seed.diagnostics[i].retained) {return 0;}
              if (seed.diagnostics[i - 1].retained || seed.diagnostics[i + 1].retained) {return 1;}
              return 2;
            };
          std::stable_sort(
            corners.begin(), corners.end(), [&](std::size_t a, std::size_t b) {
              return priority(a) < priority(b);
            });
          for (const auto i : corners) {
            for (std::size_t j = 1; j <= cfg.local_shift_samples; ++j) {
              for (double sign : {-1.0, 1.0}) {
                if (variants >= cfg.local_max_variants) {break;}
                const auto moved = relocate(
                  grid, seed.vertices, i,
                  sign * cfg.local_shift_max * j / cfg.local_shift_samples, cfg, budget);
                if (moved.size() >= 3) {consider(moved, true);}
              }
              if (variants >= cfg.local_max_variants) {break;}
            }
            if (variants >= cfg.local_max_variants) {break;}
          }
          if (variants >= cfg.local_max_variants) {break;}
        }
      }
    }
  } catch (const SearchExhausted &) {
    budget.exhausted = true;
    if (budget.reason.empty()) {budget.reason = "sampling_limit";}
  } catch (const std::length_error &) {
    budget.exhausted = true;
    budget.reason = "output_limit";
  }
  // Final geometry validation is independent of candidate selection and cost.
  for (std::size_t i = 1; i < best.size(); ++i) {
    if (!line_of_sight(grid, xy(best[i - 1]), xy(best[i]))) {
      throw std::invalid_argument("Joint result failed final collision validation");
    }
  }
  if (path_length(best) > maximum_length + kEpsilon) {
    throw std::logic_error("Joint result exceeded path length budget");
  }
  stats.selected_length = best_score.length;
  stats.estimated_time = traversal_time(best, cfg);
  stats.retained_corners = best_score.stops;
  stats.arc_fallbacks = best_score.fallbacks;
  stats.relocated = relocated;
  stats.work_used = budget.used;
  stats.variants_evaluated = variants;
  stats.budget_exhausted = budget.exhausted;
  stats.budget_reason = budget.reason;
  if (!selected_diagnostics.empty()) {
    stats.corners = std::move(selected_diagnostics);
    if (!selected_joint) {
      for (auto & c : stats.corners) {
        c.chosen = std::numeric_limits<std::size_t>::max();
        c.retained = std::any_of(
          best.begin(), best.end(), [&](const PathPoint2D & p) {
            return p.stop_required && dist({c.x, c.y}, xy(p)) <= kEpsilon;
          });
      }
    }
  } else {
    for (std::size_t i = 0; i + 1 < best.size(); ++i) {
      if (best[i].stop_required) {
        CornerDiagnostic d; d.x = best[i].x; d.y = best[i].y; d.retained = true;
        stats.corners.push_back(d);
      }
    }
  }
  stats.rounded_corners = 0;
  for (std::size_t i = 1; i < best.size(); ++i) {
    stats.rounded_corners += best[i].segment_type != 0 && best[i - 1].segment_type == 0;
  }
  stats.minimum_clearance = std::numeric_limits<double>::infinity();
  for (const auto & p : best) {
    stats.minimum_clearance = std::min(stats.minimum_clearance, grid.clearance_at(p.x, p.y));
  }
  return best;
}

std::string processing_report(const PathProcessingStats & stats)
{
  std::ostringstream out;
  out.precision(12);
  out << "{\"baseline_length\":" << stats.baseline_length << ",\"selected_length\":" <<
    stats.selected_length << ",\"estimated_time\":" << stats.estimated_time <<
    ",\"minimum_clearance\":" << stats.minimum_clearance << ",\"retained_corners\":" <<
    stats.retained_corners << ",\"arc_fallbacks\":" << stats.arc_fallbacks <<
    ",\"relocated\":" << (stats.relocated ? "true" : "false") <<
    ",\"budget_exhausted\":" << (stats.budget_exhausted ? "true" : "false") <<
    ",\"budget_reason\":\"" << stats.budget_reason << "\"" <<
    ",\"work_used\":" << stats.work_used << ",\"variants_evaluated\":" <<
    stats.variants_evaluated << ",\"corners\":[";
  bool first = true;
  for (const auto & c : stats.corners) {
    if (c.trials.empty() && !c.retained) {continue;}
    if (!first) {out << ',';} first = false;
    out << "{\"x\":" << c.x << ",\"y\":" << c.y << ",\"retained\":" <<
      (c.retained ? "true" : "false") << ",\"candidates\":" << c.candidates <<
      ",\"space\":" << c.space << ",\"collision\":" << c.collision <<
      ",\"curvature\":" << c.curvature << ",\"speed\":" << c.speed <<
      ",\"deviation\":" << c.deviation << ",\"conflicts\":" << c.conflicts <<
      ",\"length_budget\":" << c.length << ",\"chosen_candidate\":";
    if (c.chosen == std::numeric_limits<std::size_t>::max()) {out << "null";} else {
      out << c.chosen;
    }
    out << ",\"trials\":[";
    bool first_trial = true;
    for (const auto & t : c.trials) {
      if (!first_trial) {out << ',';} first_trial = false;
      out << "{\"type\":" << static_cast<unsigned>(t.kind) << ",\"radius\":" << t.radius <<
        ",\"transition\":" << t.transition << ",\"blend\":" << t.blend <<
        ",\"trim_in\":" << t.trim_in << ",\"trim_out\":" << t.trim_out <<
        ",\"speed_cap\":" << t.speed << ",\"result\":\"" << t.result << "\"}";
    }
    out << "]}";
  }
  out << "]}";
  return out.str();
}
}  // namespace plan

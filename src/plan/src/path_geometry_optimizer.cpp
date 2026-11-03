// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "path_geometry_optimizer.hpp"
#include "path_experiments.hpp"
#include "path_optimizer.hpp"
#include "path_turn_features.hpp"
#include <ceres/ceres.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <tuple>

namespace plan
{
bezier::Bezier geometry_policy_curve(const double * a, const double * b)
{
  double left[5], right[5];
  std::copy(a, a + 5, left);std::copy(b, b + 5, right);
  const double chord = std::hypot(b[0] - a[0], b[1] - a[1]);
  left[3] *= chord;right[3] *= chord;
  return bezier::from_knots(left, right);
}
namespace
{
using namespace bezier;
using Clock = std::chrono::steady_clock;
Point2D xy(const PathPoint2D & p) {return {p.x, p.y};}
double seconds(Clock::time_point t)
{
  return std::chrono::duration<double>(Clock::now() - t).count();
}
std::string num(double v)
{
  if (!std::isfinite(v)) {return "null";}
  std::ostringstream s;s << std::setprecision(16) << v;return s.str();
}
std::string json_quote(const std::string & value)
{
  std::string out = "\"";
  for (char ch:value) {
    if (ch == '"' || ch == '\\') {out += '\\';out += ch;} else if (ch == '\n') {
      out += "\\n";
    } else if (ch == '\r') {out += "\\r";} else if (ch == '\t') {out += "\\t";} else {
      out += ch;
    }
  }
  return out + '"';
}
std::string array(const std::vector<double> & a)
{
  std::string s = "[";
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (i) {
      s += ',';
    }
    s += num(a[i]);
  }
  return s + ']';
}
Point2D closest(Point2D p, Point2D a, Point2D b)
{
  const auto v = sub(b, a);const double d = dot(v, v);
  return add(a, mul(v, d > 0 ? std::clamp(dot(sub(p, a), v) / d, 0.0, 1.0) : 0.0));
}
Point2D nearest(Point2D p, const std::vector<PathPoint2D> & ref)
{
  auto best = xy(ref.front());double distance = INFINITY;
  for (std::size_t i = 1; i < ref.size(); ++i) {
    const auto q = closest(p, xy(ref[i - 1]), xy(ref[i]));
    const double d = dot(sub(p, q), sub(p, q));
    if (d < distance) {best = q;distance = d;}
  }
  return best;
}
std::vector<double> lengths(const std::vector<PathPoint2D> & p)
{
  std::vector<double> s(p.size(), 0);
  for (std::size_t i = 1; i < p.size(); ++i) {
    s[i] = s[i - 1] + dist(xy(p[i - 1]), xy(p[i]));
  }
  return s;
}
void caps(std::vector<PathPoint2D> & p, const MotionLimits & m)
{
  for (std::size_t i = 1; i < p.size(); ++i) {
    auto & a = p[i - 1];const auto & b = p[i];
    a.curvature_bound = std::max(a.curvature_bound, std::abs(a.curvature));
    if (!b.stop_required) {
      a.curvature_bound = std::max(a.curvature_bound, std::abs(b.curvature));
      a.curvature_rate_bound = std::max(
        a.curvature_rate_bound, std::abs(a.curvature - b.curvature) / dist(xy(a), xy(b)));
    }
    a.speed_limit = curve_limits(m, a.curvature_bound, a.curvature_rate_bound).speed;
  }
  p.back().stop_required = true;p.back().speed_limit = 0;
}
std::size_t stops(const std::vector<PathPoint2D> & p)
{
  return std::count_if(p.begin(), p.end() - 1, [](const auto & a) {return a.stop_required;});
}
double segment_distance(Point2D a, Point2D b, Point2D c, Point2D d)
{
  const auto u = sub(b, a), v = sub(d, c);const double den = cross(u, v);
  if (std::abs(den) > 1e-12) {
    const double t = cross(sub(c, a), v) / den, q = cross(sub(c, a), u) / den;
    if (t >= 0 && t <= 1 && q >= 0 && q <= 1) {return 0;}
  }
  return std::min(
    {dist(a, closest(a, c, d)), dist(b, closest(b, c, d)),
      dist(c, closest(c, a, b)), dist(d, closest(d, a, b))});
}
// Continuous, optimizer-only field. Its interpolation is never a safety certificate.
struct DistanceField
{
  GridGeometry g;
  std::vector<double> values;
  explicit DistanceField(const PlanningGrid & grid)
  : g(grid.geometry())
  {
    auto inverse = grid.inflated_grid();
    for (auto & value:inverse) {value = value == 100 ? 0 : 100;}
    const auto inverted =
      PlanningGrid::from_inflated_grid(g, std::move(inverse), g.width * g.height);
    values.reserve(g.width * g.height);
    for (std::size_t y = 0; y < g.height; ++y) {
      for (std::size_t x = 0; x < g.width; ++x) {
        const auto p = grid.cell_to_world(x, y);
        values.push_back(
          grid.is_blocked(x, y) ?
          -inverted.clearance_at(p.first, p.second) : grid.clearance_at(p.first, p.second));
      }
    }
  }
  double at(Point2D p) const
  {
    const double c = std::cos(g.origin_yaw), s = std::sin(g.origin_yaw);
    const double dx = p.x - g.origin_x, dy = p.y - g.origin_y;
    const double x = (c * dx + s * dy) / g.resolution - .5;
    const double y = (-s * dx + c * dy) / g.resolution - .5;
    if (x < 0 || y < 0 || x >= g.width - 1 || y >= g.height - 1) {return -g.resolution;}
    const auto i = static_cast<std::size_t>(x), j = static_cast<std::size_t>(y);
    const double a = x - i, b = y - j;
    return (1 - b) * ((1 - a) * values[j * g.width + i] + a * values[j * g.width + i + 1]) +
           b * ((1 - a) * values[(j + 1) * g.width + i] + a * values[(j + 1) * g.width + i + 1]);
  }
};
std::string envelope_cell(const PlanningGrid & grid, const Bezier & b, double envelope)
{
  const auto & g = grid.geometry();
  const auto coordinates = [&](Point2D p) {
      const double x = p.x - g.origin_x, y = p.y - g.origin_y;
      return Point2D{(std::cos(g.origin_yaw) * x + std::sin(g.origin_yaw) * y) / g.resolution,
      (-std::sin(g.origin_yaw) * x + std::cos(g.origin_yaw) * y) / g.resolution};
    };
  const auto a = coordinates(b.front()), z = coordinates(b.back());
  const double padding = envelope / g.resolution + 1e-10;
  const int x0 = std::max(0, static_cast<int>(std::floor(std::min(a.x, z.x) - padding)));
  const int y0 = std::max(0, static_cast<int>(std::floor(std::min(a.y, z.y) - padding)));
  const int x1 =
    std::min(
    static_cast<int>(g.width) - 1,
    static_cast<int>(std::floor(std::max(a.x, z.x) + padding)));
  const int y1 =
    std::min(
    static_cast<int>(g.height) - 1,
    static_cast<int>(std::floor(std::max(a.y, z.y) + padding)));
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      if (!grid.is_blocked(x, y)) {continue;}
      double enter = 0, leave = 1;
      const auto slab = [&](double start, double end, double lo, double hi) {
          if (start == end) {return start >= lo && start <= hi;}
          double first = (lo - start) / (end - start), last = (hi - start) / (end - start);
          if (first > last) {std::swap(first, last);}
          enter = std::max(enter, first);leave = std::min(leave, last);return enter <= leave;
        };
      if (slab(
          a.x, z.x, x - padding,
          x + 1 + padding) && slab(a.y, z.y, y - padding, y + 1 + padding))
      {
        return '[' + std::to_string(x) + ',' + std::to_string(y) + ']';
      }
    }
  }
  return "null";
}
std::string failure_json(
  const Bezier & b, std::size_t index, double u0, double u1, std::size_t depth,
  const Bounds & bound, const PlanningGrid & grid, bool detail)
{
  std::string s = "{\"curve\":" + std::to_string(index) + ",\"u\":[" + num(u0) + ',' + num(u1) +
    "],\"depth\":" + std::to_string(depth) + ",\"envelope\":" + num(bound.envelope) +
    ",\"K\":" + num(bound.curvature) + ",\"Q\":" + num(bound.rate);
  double sampled_k = 0, sampled_q = 0;
  for (double t:{0.0, .5, 1.0}) {
    const auto g = geometry(b, t);sampled_k = std::max(sampled_k, std::abs(g.curvature));
    sampled_q = std::max(sampled_q, std::abs(g.rate));
  }
  s += ",\"sampled_K\":" + num(sampled_k) + ",\"sampled_Q\":" + num(sampled_q);
  if (detail) {
    s += ",\"controls\":[";
    for (std::size_t i = 0; i < 6; ++i) {
      if (i) {
        s += ',';
      }
      s += '[' + num(b[i].x) + ',' + num(b[i].y) + ']';
    }
    s += "],\"envelope_blocking_cell\":" + envelope_cell(grid, b, bound.envelope);
    bool witness = false;
    for (double t:{0.0, .5, 1.0}) {
      const auto p = geometry(b, t).point;
      if (!curve_chord_is_free(grid, p, p, 0)) {
        std::size_t x = 0, y = 0;const bool inside = grid.world_to_cell(p.x, p.y, x, y);
        s += ",\"sample_witness\":[" + num(p.x) + ',' + num(p.y) +
          "],\"point_cell\":" +
          (inside ? '[' + std::to_string(x) + ',' + std::to_string(y) + ']' : "null");
        witness = true;
        break;
      }
    }
    if (!witness) {s += ",\"sample_witness\":null";}
  }
  return s + '}';
}
bool certify_leaf(
  const Bezier & b, const PlanningGrid & grid, const std::vector<PathPoint2D> & ref,
  const PathProcessingConfig & c, bool seed, std::size_t index, double u0, double u1,
  std::size_t depth, GeometryCertificate & out)
{
  if (++out.used > c.window_check_budget) {out.reason = "check_budget";return false;}
  const auto bound = bounds(b);
  const auto a = geometry(b, 0), z = geometry(b, 1), m = geometry(b, .5);
  const auto fail = [&](const std::string & reason) {
      out.reason = reason;out.failure = failure_json(
        b, index, u0, u1, depth, bound, grid,
        c.window_detailed_diagnostics);
      return false;
    };
  for (const auto & p:{a, m, z}) {
    if (!std::isfinite(p.curvature) || !std::isfinite(p.rate)) {return fail("nonregular");}
    if (!curve_chord_is_free(grid, p.point, p.point, 0)) {return fail("collision_witness");}
    if (std::abs(p.curvature) > 1 / c.arc_min_radius + 1e-10 ||
      (!seed && std::abs(p.rate) > c.max_curvature_rate + 1e-10))
    {
      return fail("sampled_curvature_or_rate");
    }
  }
  const double chord = dist(b.front(), b.back());
  const double k = std::max({bound.curvature, std::abs(a.curvature), std::abs(z.curvature)});
  const double q = std::max(
    bound.rate, chord > 0 ? std::abs(
      z.curvature - a.curvature) / chord : INFINITY);
  const auto anchor = nearest(m.point, ref);
  double deviation = 0;
  for (const auto & p:b) {
    deviation = std::max(deviation, dist(p, anchor));
  }
  const bool regular = bound.lower > 1e-10 && std::isfinite(k) && std::isfinite(q) &&
    k <= 1 / c.arc_min_radius + 1e-10 &&
    (seed || q <= c.max_curvature_rate);
  const bool small = chord <= c.resample_spacing && bound.envelope <= c.arc_max_deviation &&
    std::abs(wrap(z.yaw - a.yaw)) <= c.arc_max_angle &&
    std::abs(wrap(a.yaw - heading(b.front(), b.back()))) <= c.arc_max_angle &&
    std::abs(wrap(z.yaw - heading(b.front(), b.back()))) <= c.arc_max_angle;
  const bool free = curve_chord_is_free(grid, b.front(), b.back(), bound.envelope + 1e-10);
  const bool speed = seed || (std::isfinite(k) && std::isfinite(q) &&
    curve_limits(c.motion, k, q).speed >= c.motion.min_turn_speed);
  if (!regular || !small || !free || !speed || deviation > c.window_max_deviation) {
    if (depth >= c.curve_max_depth) {
      return fail(
        !free ? "collision_unproven" : !regular ? "curvature_or_regularity_unproven" :
        deviation >
        c.window_max_deviation ? "deviation_unproven" : !speed ? "speed_unproven" : "sampling_limit");
    }
    const auto halves = split(b);const double mid = (u0 + u1) / 2;
    return certify_leaf(halves.first, grid, ref, c, seed, index, u0, mid, depth + 1, out) &&
           certify_leaf(halves.second, grid, ref, c, seed, index, mid, u1, depth + 1, out);
  }
  if (out.points.size() + 1 >= c.max_output_points) {return fail("point_limit");}
  PathPoint2D p{a.point.x, a.point.y, a.yaw, a.yaw, a.curvature, false};
  p.segment_type = 1;p.curvature_bound = k;p.curvature_rate_bound = q;
  p.speed_limit = seed ? c.motion.v_nominal : curve_limits(c.motion, k, q).speed;
  if (out.points.empty()) {
    out.points.push_back(p);out.curve_parameter.push_back(index + u0);
  } else {
    out.points.back() = p;
  }
  out.envelopes.push_back(bound.envelope);
  double length_upper = 0;
  for (std::size_t i = 1; i < b.size(); ++i) {
    length_upper += dist(b[i - 1], b[i]);
  }
  out.length_upper.push_back(length_upper);
  p.x = z.point.x;p.y = z.point.y;p.yaw = p.tangent_yaw = z.yaw;p.curvature = z.curvature;
  p.curvature_bound = std::abs(z.curvature);p.curvature_rate_bound = 0;
  out.points.push_back(p);out.curve_parameter.push_back(index + u1);
  return true;
}
struct Span {std::size_t a, b;};
std::vector<Span> events(const std::vector<PathPoint2D> & p)
{
  std::vector<Span> out;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    if (p[i].curvature_bound <= 1e-10 && !p[i].stop_required) {continue;}
    if (!out.empty() && i <= out.back().b) {out.back().b = i + 1;} else {out.push_back({i, i + 1});}
  }
  return out;
}
std::vector<Span> windows(
  const std::vector<PathPoint2D> & p, const std::vector<double> & s,
  const PathProcessingConfig & c)
{
  std::vector<Span> merged, out;
  for (auto w:events(p)) {
    if (!merged.empty() && s[w.a] - s[merged.back().b] <= c.window_merge_gap) {
      merged.back().b = w.b;
    } else {merged.push_back(w);}
  }
  for (auto w:merged) {
    const auto a = w.a, b = w.b;
    while (w.a && s[a] - s[w.a - 1] <= c.window_extension) {--w.a;}
    while (w.b + 1 < p.size() && s[w.b + 1] - s[b] <= c.window_extension) {++w.b;}
    if (!out.empty() && w.a <= out.back().b) {out.back().b = w.b;} else {out.push_back(w);}
  }
  return out;
}
PathPoint2D sample(const std::vector<PathPoint2D> & p, const std::vector<double> & s, double q)
{
  const auto i = std::min<std::size_t>(
    p.size() - 2, std::upper_bound(
      s.begin(),
      s.end(), q) - s.begin() - 1);
  const double t = std::clamp((q - s[i]) / (s[i + 1] - s[i]), 0.0, 1.0);
  auto out = p[i];out.x += t * (p[i + 1].x - out.x);out.y += t * (p[i + 1].y - out.y);
  out.tangent_yaw += t * wrap(p[i + 1].tangent_yaw - out.tangent_yaw);
  out.curvature += t * (p[i + 1].curvature - out.curvature);return out;
}
std::vector<Bezier> curves(const std::vector<double> & p)
{
  std::vector<Bezier> out;
  for (std::size_t i = 5; i < p.size(); i += 5) {
    out.push_back(geometry_policy_curve(p.data() + i - 5, p.data() + i));
  }
  return out;
}
std::vector<double> parameters(
  const std::vector<PathPoint2D> & ref, const std::vector<double> & s,
  const std::vector<double> & knots, double scale)
{
  std::vector<double> out;
  for (std::size_t i = 0; i < knots.size(); ++i) {
    const auto p = sample(ref, s, knots[i]);
    out.insert(out.end(), {p.x, p.y, p.tangent_yaw, scale, p.curvature});
  }
  return out;
}
std::vector<double> initial_knots(
  const std::vector<PathPoint2D> & ref,
  const PathProcessingConfig & c)
{
  const auto s = lengths(ref);std::vector<double> anchors{0};
  for (std::size_t i = 1; i + 1 < ref.size(); ++i) {
    const double q0 = ref[i - 1].curvature_rate_bound, q1 = ref[i].curvature_rate_bound;
    const bool boundary = ref[i].stop_required || (q0 > 1e-8) != (q1 > 1e-8) ||
      (ref[i - 1].curvature_bound > 1e-8) != (ref[i].curvature_bound > 1e-8);
    if (boundary && s[i] - anchors.back() > 1e-8) {anchors.push_back(s[i]);}
  }
  anchors.push_back(s.back());std::vector<double> knots{0};
  for (std::size_t i = 1; i < anchors.size(); ++i) {
    const auto count =
      static_cast<std::size_t>(std::ceil(
        (anchors[i] - anchors[i - 1]) /
        c.window_knot_spacing));
    for (std::size_t j = 1; j <= count; ++j) {
      knots.push_back(anchors[i - 1] + (anchors[i] - anchors[i - 1]) * j / count);
    }
  }
  while (knots.size() < c.window_max_knots) {
    const auto b = curves(parameters(ref, s, knots, 1));bool changed = false;
    for (std::size_t i = 0; i < b.size(); ++i) {
      double error = 0;
      for (double t:{.25, .5, .75}) {
        const auto point = geometry(b[i], t).point;
        error = std::max(error, dist(point, nearest(point, ref)));
      }
      if (error > c.window_seed_fit_tolerance) {
        knots.insert(knots.begin() + i + 1, (knots[i] + knots[i + 1]) / 2);changed = true;break;
      }
    }
    if (!changed) {break;}
  }
  return knots;
}
std::vector<double> objective_parameters(const Bezier & curve, const PathProcessingConfig & c)
{
  std::vector<double> out;
  for (std::size_t i = 0; i <= c.window_objective_samples; ++i) {
    out.push_back(static_cast<double>(i) / c.window_objective_samples);
  }
  if (c.window_feature_refinement) {
    const auto dense = feature_curve_samples(curve, c);
    double peak = -1, rate = -1, peak_t = 0, rate_t = 0;
    for (double t:dense) {
      const auto g = geometry(curve, t);
      if (std::abs(g.curvature) > peak) {peak = std::abs(g.curvature);peak_t = t;}
      if (std::abs(g.rate) > rate) {rate = std::abs(g.rate);rate_t = t;}
    }
    out.push_back(peak_t);out.push_back(rate_t);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
  }
  return out;
}
struct RefinementSpan {std::size_t begin, end;double priority;};
std::vector<RefinementSpan> refinement_spans(
  const std::vector<Bezier> & curves, const PathProcessingConfig & c)
{
  std::vector<RefinementSpan> spans;
  const double k_target = 1 / c.window_target_radius;
  const double q_target = k_target / c.window_transition_scale;
  for (std::size_t i = 0; i < curves.size(); ++i) {
    double severity = 0;
    for (double t:feature_curve_samples(curves[i], c)) {
      const auto g = geometry(curves[i], t);
      severity = std::max(
        {severity, std::abs(g.curvature) / k_target,
          std::abs(g.rate) / q_target});
    }
    if (severity <= 1) {continue;}
    const std::size_t begin = i ? i - 1 : 0, end = std::min(curves.size(), i + 2);
    if (!spans.empty() && begin < spans.back().end) {
      spans.back().end = end;spans.back().priority = std::max(spans.back().priority, severity);
    } else {spans.push_back({begin, end, severity});}
  }
  std::stable_sort(
    spans.begin(), spans.end(), [](const auto & a, const auto & b) {
      return a.priority > b.priority;
    });
  return spans;
}
struct GeometricCost
{
  const DistanceField & field;
  const std::vector<PathPoint2D> & ref;
  const PathProcessingConfig & c;
  double reference_length;
  std::vector<double> parameters;
  bool operator()(double const * const * p, double * r) const
  {
    const auto b = geometry_policy_curve(p[0], p[1]);const auto d = array_derivative(b);
    const double kn = 1 / c.window_target_radius, qn = kn / c.window_transition_scale;
    std::size_t out = 0;double maximum = 0;
    for (std::size_t j = 0; j < parameters.size(); ++j) {
      const double t = parameters[j];
      const auto g = geometry(b, t);if (!std::isfinite(g.curvature) || !std::isfinite(g.rate)) {
        return false;
      }
      maximum = std::max(maximum, std::abs(g.curvature));
      const double measure = std::max(1e-10, norm(array_value(d, t))) /
        (c.window_refine_objective ? reference_length : 1.0);
      const double weight = c.window_feature_refinement ? std::sqrt(
        measure * ((j ? t - parameters[j - 1] : 0.0) +
        (j + 1 < parameters.size() ? parameters[j + 1] - t : 0.0)) / 2) :
        std::sqrt(
        measure / c.window_objective_samples *
        ((j == 0 || j == c.window_objective_samples) ? .5 : 1));
      const auto q = nearest(g.point, ref);
      r[out++] = weight * std::sqrt(c.window_curvature_weight) * g.curvature / kn;
      r[out++] = weight * std::sqrt(c.window_rate_weight) * g.rate / qn;
      r[out++] = weight * std::sqrt(c.window_deviation_weight) * (g.point.x - q.x) /
        c.window_max_deviation;
      r[out++] = weight * std::sqrt(c.window_deviation_weight) * (g.point.y - q.y) /
        c.window_max_deviation;
      r[out++] = weight * std::sqrt(c.window_clearance_weight) * std::max(
        0.0, 1 - field.at(
          g.point) / c.clearance_target);
      r[out++] = weight * std::sqrt(
        c.window_refine_objective ? c.window_peak_weight : c.window_curvature_weight) * std::max(
        0.0, std::abs(
          g.curvature) / kn - 1);
      r[out++] = weight * std::sqrt(c.window_constraint_weight) * std::max(
        0.0, std::abs(
          g.rate) / c.max_curvature_rate - c.window_constraint_fraction);
      r[out++] = weight * std::sqrt(c.window_constraint_weight) * std::max(
        0.0, std::abs(
          g.curvature) * c.arc_min_radius - c.window_constraint_fraction);
    }
    for (std::size_t j = 0; j < c.window_objective_samples; ++j) {
      const auto leaf = interval(
        b, static_cast<double>(j) / c.window_objective_samples,
        static_cast<double>(j + 1) / c.window_objective_samples);
      const double chord = std::max(1e-10, dist(leaf.front(), leaf.back()));
      const auto bound = c.window_feature_refinement ?
        fixed_bounds(leaf, chord * c.window_min_tangent_ratio) :
        bounds(leaf, chord * c.window_min_tangent_ratio);
      const double weight = std::sqrt(c.window_constraint_weight * chord);
      r[out++] = weight * std::max(
        0.0,
        bound.curvature * c.arc_min_radius -
        c.window_constraint_fraction);
      r[out++] = weight * std::max(
        0.0,
        bound.rate / c.max_curvature_rate -
        c.window_constraint_fraction);
      r[out++] = weight *
        std::max(
        0.0,
        c.window_min_tangent_ratio - bound.lower /
        std::max(1e-10, dist(leaf.front(), leaf.back())));
    }
    if (c.window_max_curvature_weight > 0) {
      // Shared epigraph variable avoids a dense residual spanning every knot.
      r[out++] = std::sqrt(c.window_constraint_weight) * std::max(0.0, maximum / kn - p[2][0]);
    }
    return true;
  }
};
struct PeakCost
{
  double weight;
  template<typename T>
  bool operator()(const T * maximum, T * residual) const
  {
    residual[0] = T(weight) * maximum[0];return true;
  }
};
struct JoinCost
{
  double scale;
  bool operator()(double const * const * p, double * r) const
  {
    r[0] = scale *
      (geometry(
        geometry_policy_curve(p[0], p[1]),
        1).rate - geometry(geometry_policy_curve(p[1], p[2]), 0).rate);
    return std::isfinite(r[0]);
  }
};
struct Metrics
{
  double curvature = 0, rate = 0, peak = 0, maximum = 0;
  double deviation = 0, clearance = 0, join = 0, length = 0;
  double total() const {return curvature + rate + peak + maximum + deviation + clearance + join;}
  std::string json() const
  {
    return "{\"curvature\":" + num(curvature) + ",\"rate\":" + num(rate) + ",\"peak\":" +
           num(peak) + ",\"maximum\":" + num(maximum) +
           ",\"deviation\":" + num(deviation) + ",\"clearance\":" + num(clearance) + ",\"join\":" +
           num(join) +
           ",\"length\":" + num(length) + ",\"total\":" + num(total()) + '}';
  }
};
struct RegionQuality {double curvature = 0, rate = 0, rms = 0;};
RegionQuality region_quality(
  const std::vector<PathPoint2D> & path, const std::vector<double> & mapping,
  double begin, double end)
{
  RegionQuality out;double length = 0, cost = 0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double overlap = std::min(end, mapping[i + 1]) - std::max(begin, mapping[i]);
    if (overlap <= 1e-10) {continue;}
    const double ds = dist(xy(path[i]), xy(path[i + 1]));
    const double fraction = overlap / (mapping[i + 1] - mapping[i]);
    out.curvature = std::max(out.curvature, path[i].curvature_bound);
    out.rate = std::max(out.rate, path[i].curvature_rate_bound);
    cost += path[i + 1].stop_required ? 0.0 :
      std::pow(path[i + 1].curvature - path[i].curvature, 2) / ds * fraction;
    length += ds * fraction;
  }
  out.rms = length > 0 ? std::sqrt(cost / length) : 0;
  return out;
}
bool feature_regions_not_worse(
  const std::vector<PathPoint2D> & candidate, const std::vector<double> & source,
  const std::vector<PathPoint2D> & previous, const std::vector<double> & previous_source,
  const TurnFeatures & features, const PathProcessingConfig & config)
{
  std::vector<double> edges{0.0};
  for (std::size_t i = 1; i < features.turns.size(); ++i) {
    // Strongly coupled turns share a guard region, including their transition
    // margin. Do not freeze tiny low-curvature subintervals inside an S bend.
    if (features.turns[i].begin - features.turns[i - 1].end > config.window_merge_gap) {
      edges.push_back((features.turns[i - 1].end + features.turns[i].begin) / 2);
    }
  }
  edges.push_back(previous_source.back());
  for (std::size_t i = 1; i < edges.size(); ++i) {
    if (edges[i] - edges[i - 1] <= 1e-10) {continue;}
    const auto a = region_quality(previous, previous_source, edges[i - 1], edges[i]);
    const auto b = region_quality(candidate, source, edges[i - 1], edges[i]);
    if (b.curvature > a.curvature + 1e-8 || b.rate > a.rate + 1e-8 || b.rms > a.rms + 1e-8) {
      return false;
    }
  }
  return true;
}
Metrics metrics(
  const std::vector<PathPoint2D> & path, const std::vector<PathPoint2D> & ref,
  const PathProcessingConfig & c, const DistanceField & field, bool baseline = false)
{
  Metrics m;double maximum = 0;const double denominator = lengths(ref).back();
  const double kn = 1 / c.window_target_radius, qn = kn / c.window_transition_scale;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1];const auto & b = path[i];const double ds = dist(xy(a), xy(b));
    const double rate = b.stop_required ? 0 : (b.curvature - a.curvature) / ds;
    m.length += ds;
    for (double t:{0.0, .5, 1.0}) {
      const double w = ds / denominator * (t == .5 ? 4.0 : 1.0) / 6.0;
      const double k = a.curvature + t * (b.curvature - a.curvature);
      const auto p = add(xy(a), mul(sub(xy(b), xy(a)), t));const auto q = nearest(p, ref);
      m.curvature += w * c.window_curvature_weight * std::pow(k / kn, 2);
      m.rate += w * c.window_rate_weight * std::pow(rate / qn, 2);
      maximum = std::max(maximum, std::abs(k));
      m.peak += w * (c.window_refine_objective ? c.window_peak_weight : c.window_curvature_weight) *
        std::pow(std::max(0.0, std::abs(k) / kn - 1), 2);
      m.deviation += w * c.window_deviation_weight * dot(sub(p, q), sub(p, q)) / std::pow(
        c.window_max_deviation, 2);
      m.clearance += w * c.window_clearance_weight *
        std::pow(std::max(0.0, 1 - field.at(p) / c.clearance_target), 2);
    }
  }
  if (baseline) {
    for (std::size_t i = 1; i + 1 < path.size(); ++i) {
      if (path[i].stop_required || path[i + 1].stop_required) {continue;}
      const double a = (path[i].curvature - path[i - 1].curvature) /
        dist(xy(path[i - 1]), xy(path[i]));
      const double b = (path[i + 1].curvature - path[i].curvature) /
        dist(xy(path[i]), xy(path[i + 1]));
      m.join += c.window_join_weight * std::pow((a - b) / qn, 2) / denominator;
    }
  }
  m.maximum = c.window_max_curvature_weight * std::pow(maximum / kn, 2);
  return m;
}
Metrics curve_metrics(
  const std::vector<Bezier> & curves, const std::vector<PathPoint2D> & ref,
  const PathProcessingConfig & c, const DistanceField & field)
{
  Metrics m;double maximum = 0;const double denominator = lengths(ref).back();
  const double kn = 1 / c.window_target_radius, qn = kn / c.window_transition_scale;
  for (const auto & b:curves) {
    const auto derivative = array_derivative(b);
    const auto parameters = objective_parameters(b, c);
    for (std::size_t j = 0; j < parameters.size(); ++j) {
      const double t = parameters[j];
      const auto g = geometry(b, t);const auto q = nearest(g.point, ref);
      const double dt = c.window_feature_refinement ?
        ((j ? t - parameters[j - 1] : 0.0) +
        (j + 1 < parameters.size() ? parameters[j + 1] - t : 0.0)) / 2 :
        ((j == 0 || j == c.window_objective_samples) ? .5 : 1) / c.window_objective_samples;
      const double ds = c.window_feature_refinement ? norm(array_value(derivative, t)) * dt :
        norm(array_value(derivative, t)) / c.window_objective_samples *
        ((j == 0 || j == c.window_objective_samples) ? .5 : 1);
      const double w = ds / denominator;m.length += ds;
      m.curvature += w * c.window_curvature_weight * std::pow(g.curvature / kn, 2);
      m.rate += w * c.window_rate_weight * std::pow(g.rate / qn, 2);
      maximum = std::max(maximum, std::abs(g.curvature));
      m.peak += w * (c.window_refine_objective ? c.window_peak_weight : c.window_curvature_weight) *
        std::pow(std::max(0.0, std::abs(g.curvature) / kn - 1), 2);
      m.deviation += w * c.window_deviation_weight *
        dot(sub(g.point, q), sub(g.point, q)) / std::pow(c.window_max_deviation, 2);
      m.clearance += w * c.window_clearance_weight *
        std::pow(std::max(0.0, 1 - field.at(g.point) / c.clearance_target), 2);
    }
  }
  m.maximum = c.window_max_curvature_weight * std::pow(maximum / kn, 2);
  return m;
}
// Use the same number of uniformly spaced arc-length samples for both sides of a comparison.
Metrics selection_metrics(
  const std::vector<PathPoint2D> & path, const std::vector<PathPoint2D> & ref,
  const PathProcessingConfig & c, const DistanceField & field)
{
  const auto s = lengths(path);
  const auto count = std::max<std::size_t>(2, std::ceil(lengths(ref).back() / c.resample_spacing));
  std::vector<PathPoint2D> uniform;
  for (std::size_t j = 0; j <= count; ++j) {
    uniform.push_back(sample(path, s, s.back() * j / count));
  }
  return metrics(uniform, ref, c, field, true);
}
std::string geometry_summary(const std::vector<PathPoint2D> & p, const PlanningGrid & grid)
{
  double k = 0, q = 0, v = INFINITY, clearance = INFINITY;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    k = std::max(k, p[i].curvature_bound);q = std::max(q, p[i].curvature_rate_bound);
    v = std::min(v, p[i].speed_limit);
    clearance = std::min(clearance, grid.clearance_at(p[i].x, p[i].y));
  }
  return "{\"peak_curvature_bound\":" + num(k) + ",\"peak_rate_bound\":" + num(q) +
         ",\"minimum_effective_radius\":" + num(k > 0 ? 1 / k : 0) + ",\"bottleneck_speed\":" + num(
    v) +
         ",\"sampled_minimum_clearance\":" + num(clearance) + '}';
}
}  // namespace

GeometryCertificate certify_geometry(
  const PlanningGrid & grid, const std::vector<Bezier> & b,
  const std::vector<PathPoint2D> & ref, const PathProcessingConfig & c,
  bool seed, std::size_t used)
{
  GeometryCertificate out;out.used = used;
  for (std::size_t i = 0; i < b.size(); ++i) {
    if (!certify_leaf(b[i], grid, ref, c, seed, i, 0, 1, 0, out)) {return out;}
  }
  out.safe = !out.points.empty();out.reason = out.safe ? "certified" : "empty_candidate";return out;
}
std::string geometry_fingerprint(const std::vector<PathPoint2D> & p)
{
  std::uint64_t hash = 14695981039346656037ULL;
  const auto real = [&](double value) {
      std::uint64_t bits;std::memcpy(&bits, &value, sizeof(bits));
      for (int i = 0; i < 8; ++i) {
        hash ^= (bits >> (8 * i)) & 255;hash *= 1099511628211ULL;
      }
    };
  for (const auto & a:p) {
    real(a.x);real(a.y);real(a.tangent_yaw);real(a.curvature);real(a.stop_required ? 1.0 : 0.0);
  }
  std::ostringstream s;s << std::hex << std::setw(16) << std::setfill('0') << hash;return s.str();
}

std::vector<PathPoint2D> optimize_window_geometry(
  const PlanningGrid & grid, const std::vector<PathPoint2D> & input,
  const PathProcessingConfig & original, PathProcessingStats & stats)
{
  auto c = original;c.motion.enable_dynamic_angular_budget = false;
  c.enable_turn_speed_coordination = false;
  auto frozen = input;caps(frozen, c.motion);auto best = frozen;
  const auto frozen_s = lengths(frozen);auto mapping = frozen_s;
  std::vector<double> length_bounds(frozen.size(), 0);
  for (std::size_t i = 1; i < frozen.size(); ++i) {
    length_bounds[i - 1] = frozen_s[i] - frozen_s[i - 1];
  }
  const double l0 = frozen_s.back(), t0 = legacy_executable_time(frozen, c);
  stats.baseline_length = l0;stats.frozen_baseline_time = t0;
  stats.geometry_policy = "geometry_first";
  const auto begun = Clock::now();const DistanceField field(grid);
  std::vector<std::string> reports;std::size_t next_id = 0;
  const auto ev = events(frozen);
  std::function<void(Span, std::size_t, long, double, std::size_t &)> optimize;
  optimize = [&](Span w, std::size_t depth, long parent, double allocation, std::size_t & checks) {
      auto window_config = c;
      const auto started = Clock::now();const auto id = next_id++;
      const std::vector<PathPoint2D> ref(frozen.begin() + w.a, frozen.begin() + w.b + 1);
      const auto ss = lengths(ref);
      double original_peak = 0;
      for (const auto & q:ref) {
        original_peak = std::max(original_peak, q.curvature_bound);
      }
      window_config.window_feature_refinement = window_config.window_feature_refinement &&
        original_peak > window_config.window_feature_min_curvature + 1e-10;
      const auto features = window_config.window_feature_refinement ?
        path_turn_features(ref, window_config) : TurnFeatures{};
      window_config.window_feature_refinement = window_config.window_feature_refinement &&
        features.turns.size() >= window_config.window_feature_min_turns;
      const auto window_events = static_cast<std::size_t>(std::count_if(
          ev.begin(), ev.end(),
          [&](auto e) {return e.a >= w.a && e.b <= w.b;}));
      if ((window_config.window_refine_min_curvature > 0 &&
        original_peak <= window_config.window_refine_min_curvature + 1e-10) ||
        window_events < window_config.window_refine_min_events)
      {
        window_config.window_refine_objective = false;window_config.window_max_curvature_weight = 0;
        window_config.window_prioritize_coarse_seed = false;
        window_config.window_local_refinement = false;
      }
      if (window_config.window_refine_objective && window_config.window_refine_rate_weight >= 0) {
        window_config.window_rate_weight = window_config.window_refine_rate_weight;
      }
      if (window_config.window_feature_refinement) {
        window_config.window_refine_objective = true;
        window_config.window_max_curvature_weight = std::max(
          window_config.window_max_curvature_weight, window_config.window_peak_weight);
        window_config.window_rate_weight = window_config.window_feature_rate_weight;
      }
      auto objective_config = window_config;
      if (window_config.window_refine_objective && original_peak > 0) {
        objective_config.arc_min_radius = std::max(window_config.arc_min_radius, 1 / original_peak);
      }
      auto preserved_objective = objective_config;
      preserved_objective.window_feature_refinement = false;
      preserved_objective.window_max_curvature_weight = c.window_max_curvature_weight;
      preserved_objective.window_rate_weight = c.window_refine_rate_weight >= 0 ?
        c.window_refine_rate_weight : c.window_rate_weight;
      const auto base_metrics = window_config.window_refine_objective ? selection_metrics(
        ref, ref,
        window_config,
        field) :
        metrics(ref, ref, window_config, field, true);
      std::size_t cut = w.a;std::tuple<int, double, double> cut_score{2, INFINITY, INFINITY};
      if (depth < window_config.window_split_max_depth) {
        for (std::size_t i = w.a + 1; i < w.b; ++i) {
          if (frozen_s[i] - frozen_s[w.a] < window_config.window_split_min_length ||
            frozen_s[w.b] - frozen_s[i] < window_config.window_split_min_length) {continue;}
          const auto left = std::count_if(
            ev.begin(), ev.end(), [&](auto e) {
              return e.a >= w.a && e.b <= i;
            });
          const auto right = std::count_if(
            ev.begin(), ev.end(), [&](auto e) {
              return e.a >= i && e.b <= w.b;
            });
          if (left < static_cast<long>(window_config.window_split_min_events) ||
            right < static_cast<long>(window_config.window_split_min_events)) {continue;}
          const auto score = std::make_tuple(
            frozen[i].curvature_bound > 1e-10 ? 1 : 0,
            frozen[i].curvature_rate_bound,
            std::abs(frozen_s[i] - (frozen_s[w.a] + frozen_s[w.b]) / 2));
          if (score < cut_score) {cut_score = score;cut = i;}
        }
      }
      const bool splittable = cut > w.a;
      double parent_budget = allocation *
        (splittable ? window_config.window_parent_budget_fraction : 1.0);
      const auto fit_started = Clock::now();
      const auto fitted_knots = initial_knots(ref, window_config);
      const double fit_seconds = seconds(fit_started);
      const auto coarse_index = static_cast<std::size_t>(std::min_element(
          window_config.window_seed_scales.begin(), window_config.window_seed_scales.end(),
          [](double a, double b) {
            return std::abs(a - 1) < std::abs(b - 1);
          }) - window_config.window_seed_scales.begin());
      const auto current_range = [&]() {
          const auto a = std::lower_bound(mapping.begin(), mapping.end(), frozen_s[w.a] - 1e-10);
          const auto b = std::lower_bound(mapping.begin(), mapping.end(), frozen_s[w.b] - 1e-10);
          return Span{static_cast<std::size_t>(a - mapping.begin()),
          static_cast<std::size_t>(b - mapping.begin())};
        };
      bool adopted = false;double winning = base_metrics.total();
      auto selected_points = ref;std::vector<double> selected_knots, selected_reference_knots;
      auto selected_source = ss;
      std::string candidates = "[", search_end = "completed", selected = "null";
      std::size_t attempts = 0;
      std::vector<std::size_t> seed_order;
      for (std::size_t i = 0; i < window_config.window_seed_scales.size(); ++i) {
        seed_order.push_back(i);
      }
      if (window_config.window_prioritize_coarse_seed && window_config.window_include_coarse_seed) {
        std::rotate(
          seed_order.begin(), seed_order.begin() + coarse_index,
          seed_order.begin() + coarse_index + 1);
      }
      const auto ordinary_trials = seed_order.size();
      if (window_config.window_feature_refinement) {
        // Keep the existing coarse/fitted seeds as alternatives to the feature seed.
        seed_order.insert(seed_order.begin() + 1, ordinary_trials + 1);
      }
      if (window_config.window_local_refinement) {
        seed_order.insert(
          seed_order.begin() + (window_config.window_prioritize_coarse_seed ? 1 :
          ordinary_trials), window_config.window_seed_scales.size());
        if (window_config.window_feature_refinement) {
          for (std::size_t pass = 1; pass < window_config.window_feature_max_passes; ++pass) {
            seed_order.push_back(ordinary_trials + 1 + pass);
          }
        }
      }
      std::string refinement_status =
        window_config.window_local_refinement ? "not_attempted" : "disabled";
      for (std::size_t order = 0; order < seed_order.size(); ++order) {
        const auto seed_index = seed_order[order];
        const bool feature_seed = seed_index == ordinary_trials + 1;
        const bool local = seed_index == ordinary_trials || seed_index > ordinary_trials + 1;
        const bool preserve_seed = window_config.window_feature_refinement && order == 0 &&
          seed_index == coarse_index;
        const auto & solve_config = preserve_seed ? preserved_objective : objective_config;
        const auto ordinary_remaining = std::count_if(
          seed_order.begin() + order, seed_order.end(),
          [&](auto i) {return i < ordinary_trials || i == ordinary_trials + 1;});
        if (local && (!adopted || selected_knots.empty())) {
          refinement_status = "no_adopted_curve";continue;
        }
        if (seconds(begun) >= window_config.window_total_seconds ||
          seconds(started) >= parent_budget)
        {
          search_end = "solver_budget";
          if (local) {refinement_status = "no_remaining_parent_budget";}
          break;
        }
        auto knots = local ? selected_reference_knots : fitted_knots;
        const bool coarse = feature_seed || (!local && window_config.window_include_coarse_seed &&
          seed_index == coarse_index);
        const double seed_scale = local || feature_seed ? 1.0 :
          window_config.window_seed_scales[seed_index];
        const std::string seed_type =
          preserve_seed ? "preserved_coarse" : local ? "local_refinement" : feature_seed ?
          "feature_spaced" :
          coarse ? "coarse_spaced" : "baseline_fit";
        if (coarse) {
          knots.clear();
          const auto count =
            static_cast<std::size_t>(std::ceil(ss.back() / window_config.window_knot_spacing));
          for (std::size_t j = 0; j <= count; ++j) {
            knots.push_back(ss.back() * j / count);
          }
          if (feature_seed) {
            std::vector<double> anchors = features.zero_crossings;
            for (const auto & turn:features.turns) {
              anchors.push_back(turn.peak_s);
            }
            for (double anchor:anchors) {
              const auto next = std::lower_bound(knots.begin(), knots.end(), anchor);
              if (next == knots.end() || next == knots.begin()) {continue;}
              if (std::min(*next - anchor, anchor - *(next - 1)) >=
                window_config.window_feature_knot_min_spacing)
              {
                knots.insert(next, anchor);
              }
            }
          }
        }
        if (knots.size() > window_config.window_max_knots) {
          if (attempts++) {candidates += ',';}
          candidates += "{\"id\":" + std::to_string(seed_index) + ",\"seed_scale\":" + num(
            seed_scale) +
            ",\"seed_type\":\"" + seed_type +
            "\",\"seed_validation\":\"not_checked\",\"result\":\"knot_limit\",\"iterations\":0,\"knots_reference_s\":"
            +
            array(knots) + '}';
          continue;
        }
        const auto candidate_start = Clock::now();
        auto p = local ? selected_knots : parameters(ref, ss, knots, seed_scale);
        std::size_t active_begin = 0, active_end = knots.size() - 1;
        if (local) {
          if (window_config.window_feature_refinement) {
            const auto spans = refinement_spans(curves(p), window_config);
            const auto pass = seed_index == ordinary_trials ? 0 : seed_index - ordinary_trials - 1;
            if (pass >= spans.size()) {refinement_status = "no_remaining_hotspot_group";continue;}
            active_begin = spans[pass].begin;active_end = spans[pass].end;
            refinement_status = "attempted";
          } else {
            double peak = 0;std::size_t peak_segment = 0;
            const auto selected_curves = curves(p);
            for (std::size_t j = 0; j < selected_curves.size(); ++j) {
              for (std::size_t k = 0; k <= window_config.window_objective_samples; ++k) {
                const double value = std::abs(
                  geometry(
                    selected_curves[j],
                    static_cast<double>(k) / window_config.window_objective_samples).curvature);
                if (value > peak) {peak = value;peak_segment = j;}
              }
            }
            if (peak <= 1 / window_config.window_target_radius) {
              refinement_status = "no_peak_over_target";continue;
            }
            active_begin = peak_segment ? peak_segment - 1 : 0;
            active_end = std::min(knots.size() - 1, peak_segment + 2);
            refinement_status = "attempted";
          }
        }
        const auto initial = p;const auto seed_started = Clock::now();
        auto seed_check = certify_geometry(
          grid, curves(
            p), ref, window_config, true, checks);
        checks = seed_check.used;const double seed_seconds = seconds(seed_started);
        const auto skipped = [&](const std::string & reason) {
            if (attempts++) {candidates += ',';}
            candidates += "{\"id\":" + std::to_string(seed_index) +
              ",\"seed_type\":" + json_quote(seed_type) +
              ",\"seed_scale\":" + num(seed_scale) +
              ",\"result\":" + json_quote(reason) +
              ",\"seed_validation\":" + json_quote(seed_check.reason) +
              ",\"iterations\":0,\"seed_seconds\":" + num(seed_seconds) + '}';
            if (local) {refinement_status = reason;}
          };
        if (window_config.window_prioritize_coarse_seed && (seconds(started) >= parent_budget ||
          seconds(begun) >= window_config.window_total_seconds ||
          checks >= window_config.window_check_budget))
        {
          search_end = checks >=
            window_config.window_check_budget ? "check_budget" : "solver_budget";
          skipped(search_end + "_before_residuals");break;
        }
        const auto residual_started = Clock::now();
        ceres::Problem problem;double maximum_parameter = 0;
        if (solve_config.window_max_curvature_weight > 0) {
          for (const auto & b:curves(p)) {
            for (std::size_t j = 0; j <= window_config.window_objective_samples; ++j) {
              maximum_parameter = std::max(
                maximum_parameter, std::abs(
                  geometry(
                    b,
                    static_cast<double>(j) / window_config.window_objective_samples).curvature) *
                window_config.window_target_radius);
            }
          }
          problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<PeakCost, 1, 1>(
              new PeakCost{std::sqrt(
                  solve_config.window_max_curvature_weight)}), nullptr, &maximum_parameter);
          problem.SetParameterLowerBound(&maximum_parameter, 0, 0);
        }
        // Per-segment references avoid repeated searches over an entire long window.
        std::vector<std::vector<PathPoint2D>> local_refs;
        local_refs.reserve(knots.size() - 1);
        for (std::size_t j = 1; j < knots.size(); ++j) {
          const auto lo =
            std::lower_bound(
            ss.begin(), ss.end(), std::max(
              0.0,
              knots[j - 1] - window_config.window_knot_spacing));
          const auto hi =
            std::upper_bound(
            ss.begin(), ss.end(), std::min(
              ss.back(), knots[j] + window_config.window_knot_spacing));
          local_refs.emplace_back(ref.begin() + (lo - ss.begin()), ref.begin() + (hi - ss.begin()));
          const auto parameters = objective_parameters(
            geometry_policy_curve(p.data() + 5 * (j - 1), p.data() + 5 * j), solve_config);
          auto * cost = new ceres::DynamicNumericDiffCostFunction<GeometricCost, ceres::CENTRAL>(
            new GeometricCost{field, local_refs.back(), solve_config, ss.back(), parameters});
          cost->AddParameterBlock(5);cost->AddParameterBlock(5);
          if (solve_config.window_max_curvature_weight > 0) {cost->AddParameterBlock(1);}
          cost->SetNumResiduals(
            static_cast<int>(8 * parameters.size() + 3 *
            window_config.window_objective_samples +
            (solve_config.window_max_curvature_weight > 0 ? 1 : 0)));
          std::vector<double *> blocks{p.data() + 5 * (j - 1), p.data() + 5 * j};
          if (solve_config.window_max_curvature_weight > 0) {blocks.push_back(&maximum_parameter);}
          problem.AddResidualBlock(cost, nullptr, blocks);
          if (j > 1) {
            auto * join = new ceres::DynamicNumericDiffCostFunction<JoinCost, ceres::CENTRAL>(
              new JoinCost{
              std::sqrt(
                window_config.window_join_weight /
                (window_config.window_refine_objective ? ss.back() : 1.0)) *
              window_config.window_target_radius * window_config.window_transition_scale});
            for (int k = 0; k < 3; ++k) {
              join->AddParameterBlock(5);
            }
            join->SetNumResiduals(1);
            problem.AddResidualBlock(
              join, nullptr, p.data() + 5 * (j - 2),
              p.data() + 5 * (j - 1), p.data() + 5 * j);
          }
        }
        for (std::size_t j = 0; j < knots.size(); ++j) {
          auto * block = p.data() + 5 * j;
          if (local && (j <= active_begin || j >= active_end)) {
            // Fix entire boundary knots so untouched neighbouring curves are identical.
            problem.SetParameterBlockConstant(block);
          } else if (!j || j + 1 == knots.size()) {
            problem.SetParameterization(block, new ceres::SubsetParameterization(5, {0, 1, 2, 4}));
          }
          for (int k:{0, 1}) {
            problem.SetParameterLowerBound(
              block, k, initial[5 * j + k] - window_config.window_max_deviation / std::sqrt(
                2.0));
            problem.SetParameterUpperBound(
              block, k, initial[5 * j + k] + window_config.window_max_deviation / std::sqrt(
                2.0));
          }
          problem.SetParameterLowerBound(
            block, 2,
            initial[5 * j + 2] - window_config.window_heading_max_change);
          problem.SetParameterUpperBound(
            block, 2,
            initial[5 * j + 2] + window_config.window_heading_max_change);
          problem.SetParameterLowerBound(
            block, 3,
            initial[5 * j + 3] * window_config.window_derivative_min_scale);
          problem.SetParameterUpperBound(
            block, 3,
            initial[5 * j + 3] * window_config.window_derivative_max_scale);
          problem.SetParameterLowerBound(block, 4, -1 / window_config.arc_min_radius);
          problem.SetParameterUpperBound(block, 4, 1 / window_config.arc_min_radius);
        }
        const double residual_seconds = seconds(residual_started);
        if (window_config.window_prioritize_coarse_seed && (seconds(started) >= parent_budget ||
          seconds(begun) >= window_config.window_total_seconds))
        {
          search_end = "solver_budget";skipped("solver_budget_before_solve");break;
        }
        ceres::Solver::Options options;options.num_threads = 1;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
        options.max_num_iterations = window_config.window_iterations;
        options.max_solver_time_in_seconds = std::max(
          1e-6, std::min(
            window_config.window_total_seconds - seconds(begun),
            (parent_budget - seconds(started)) /
            (local ? (ordinary_remaining ? 2.0 : 1.0) :
            window_config.window_prioritize_coarse_seed && coarse && order == 0 ?
            1.0 / window_config.window_coarse_budget_fraction : ordinary_remaining)));
        // A completed early solve leaves time for later seeds and the optional local pass.
        const auto solver_started = Clock::now();
        ceres::Solver::Summary summary;ceres::Solve(options, &problem, &summary);
        const double solver_seconds = seconds(solver_started);
        std::string result = "solver_failure";Metrics candidate_metrics;
        double certificate_seconds = 0, scoring_seconds = 0;
        double candidate_time = NAN, candidate_length = NAN, candidate_events = NAN;
        GeometryCertificate cert;const auto range = current_range();
        const bool can_check = !window_config.window_prioritize_coarse_seed ||
          (seconds(started) < parent_budget && seconds(begun) < window_config.window_total_seconds);
        if (!can_check) {result = "solver_budget_before_certificate";}
        if (summary.IsSolutionUsable() && can_check) {
          auto constrained = window_config;double old_peak = 0;
          for (const auto & q:ref) {
            old_peak = std::max(old_peak, q.curvature_bound);
          }
          if (old_peak > 0) {
            constrained.arc_min_radius = std::max(window_config.arc_min_radius, 1 / old_peak);
          }
          const auto certificate_started = Clock::now();
          cert = certify_geometry(grid, curves(p), ref, constrained, false, checks);
          certificate_seconds = seconds(certificate_started);
          const auto scoring_started = Clock::now();
          checks = cert.used;
          result = cert.reason;
          if (cert.safe) {
            bool free = true;
            std::vector<double> candidate_source;
            for (double q:cert.curve_parameter) {
              const auto i = std::min<std::size_t>(knots.size() - 2, static_cast<std::size_t>(q));
              candidate_source.push_back(knots[i] + (q - i) * (knots[i + 1] - knots[i]));
            }
            if (window_config.window_feature_refinement && !feature_regions_not_worse(
                cert.points, candidate_source, selected_points, selected_source, features,
                window_config))
            {
              free = false;result = "feature_region_regression";
            }
            for (std::size_t i = 0; i < cert.envelopes.size() && free; ++i) {
              for (std::size_t j = i + 2; j < cert.envelopes.size(); ++j) {
                if (segment_distance(
                    xy(cert.points[i]), xy(cert.points[i + 1]), xy(cert.points[j]),
                    xy(cert.points[j + 1])) <=
                  cert.envelopes[i] + cert.envelopes[j] + 1e-10)
                {
                  free = false;result = "self_intersection_unproven";break;
                }
              }
              for (std::size_t j = 0; j + 1 < best.size() && free; ++j) {
                if ((j >= range.a && j < range.b) || (j + 1 == range.a && i == 0) ||
                  (j == range.b && i + 1 == cert.envelopes.size())) {continue;}
                if (segment_distance(
                    xy(cert.points[i]), xy(cert.points[i + 1]), xy(best[j]),
                    xy(best[j + 1])) <= cert.envelopes[i] + 1e-10)
                {
                  free = false;result = "outside_intersection_unproven";
                }
              }
            }
            candidate_metrics = curve_metrics(curves(p), ref, window_config, field);
            for (std::size_t j = 1; j + 1 < knots.size(); ++j) {
              candidate_metrics.join += window_config.window_join_weight * std::pow(
                (geometry(geometry_policy_curve(p.data() + 5 * (j - 1), p.data() + 5 * j), 1).rate -
                geometry(
                  geometry_policy_curve(
                    p.data() + 5 * j,
                    p.data() + 5 * (j + 1)), 0).rate) * window_config.window_target_radius * window_config.window_transition_scale,
                2) / ss.back();
            }
            if (window_config.window_refine_objective && !window_config.window_feature_refinement) {
              candidate_metrics = selection_metrics(cert.points, ref, window_config, field);
            }
            if (local) {
              double old_k = 0, old_q = 0, new_k = 0, new_q = 0;
              for (const auto & point:selected_points) {
                old_k = std::max(old_k, point.curvature_bound);
                old_q = std::max(old_q, point.curvature_rate_bound);
              }
              for (const auto & point:cert.points) {
                new_k = std::max(new_k, point.curvature_bound);
                new_q = std::max(new_q, point.curvature_rate_bound);
              }
              const auto previous = selection_metrics(selected_points, ref, window_config, field);
              if (new_k > old_k + 1e-10 || new_q > old_q + 1e-10 ||
                candidate_metrics.rate > previous.rate + 1e-10)
              {
                free = false;result = "local_geometry_regression";
              }
            }
            if (free) {
              auto candidate = best;auto source = mapping;auto candidate_bounds = length_bounds;
              cert.points.back() = best[range.b];
              std::vector<double> local_source;
              for (double q:cert.curve_parameter) {
                const auto i = std::min<std::size_t>(knots.size() - 2, static_cast<std::size_t>(q));
                local_source.push_back(
                  frozen_s[w.a] + knots[i] + (q - i) * (knots[i + 1] - knots[i]));
              }
              candidate.erase(candidate.begin() + range.a, candidate.begin() + range.b + 1);
              candidate.insert(candidate.begin() + range.a, cert.points.begin(), cert.points.end());
              source.erase(source.begin() + range.a, source.begin() + range.b + 1);
              source.insert(source.begin() + range.a, local_source.begin(), local_source.end());
              auto local_bounds = cert.length_upper;local_bounds.push_back(length_bounds[range.b]);
              candidate_bounds.erase(
                candidate_bounds.begin() + range.a,
                candidate_bounds.begin() + range.b + 1);
              candidate_bounds.insert(
                candidate_bounds.begin() + range.a,
                local_bounds.begin(), local_bounds.end());
              candidate_length = std::accumulate(
                candidate_bounds.begin(),
                candidate_bounds.end(), 0.0);
              candidate_time = executable_time(candidate, window_config.motion);
              candidate_events = path_speed_events(candidate, window_config);
              if (candidate.size() > window_config.max_output_points ||
                candidate_length > l0 * window_config.max_length_ratio ||
                candidate_time > t0 * window_config.window_max_time_ratio)
              {
                result = "length_or_time";
              } else if (stops(candidate) > stops(best)) {
                result = "stop_count";
              } else if (stops(candidate) == stops(best) && candidate_metrics.total() >=
                winning * (adopted ? 1.0 : 1 - window_config.window_geometry_min_gain))
              {
                result = "geometry_score";
              } else {
                best = std::move(candidate);mapping = std::move(source);length_bounds = std::move(
                  candidate_bounds);winning = candidate_metrics.total();
                selected_points = cert.points;selected_knots = p;
                selected_source = std::move(candidate_source);
                selected_reference_knots = knots;
                adopted = true;result = "accepted";selected = std::to_string(seed_index);
                if (window_config.window_feature_refinement) {
                  // Accepted parents never use child fallback. Reuse its reserved
                  // share for bounded warm refinement, within the original allocation.
                  parent_budget = allocation;
                }
              }
            }
          }
          scoring_seconds = seconds(scoring_started);
        }
        if (local) {refinement_status = result;}
        if (attempts++) {candidates += ',';}
        candidates += "{\"id\":" + std::to_string(seed_index) + ",\"seed_scale\":" + num(
          seed_scale) +
          ",\"seed_type\":\"" + seed_type +
          "\",\"knots_reference_s\":" + array(knots) +
          ",\"seed_validation\":\"" + seed_check.reason + "\",\"result\":\"" + result +
          "\",\"solver_termination\":" + std::to_string(summary.termination_type) +
          ",\"solver_termination_name\":" +
          json_quote(ceres::TerminationTypeToString(summary.termination_type)) +
          ",\"solver_message\":" + json_quote(summary.message) +
          ",\"solver_time_budget\":" + num(options.max_solver_time_in_seconds) +
          ",\"iterations\":" + std::to_string(summary.iterations.size()) + ",\"initial_cost\":" +
          num(summary.initial_cost) +
          ",\"final_cost\":" + num(summary.final_cost) + ",\"metrics\":" +
          candidate_metrics.json() +
          ",\"candidate_time\":" + num(candidate_time) + ",\"candidate_length\":" + num(
          candidate_length) +
          ",\"fixed_speed_events\":" + num(candidate_events) +
          ",\"seconds\":" + num(seconds(candidate_start)) + ",\"checks\":" +
          std::to_string(checks) +
          ",\"solver_seconds\":" + num(solver_seconds) +
          ",\"seed_seconds\":" + num(seed_seconds) +
          ",\"residual_seconds\":" + num(residual_seconds) +
          ",\"certificate_seconds\":" + num(certificate_seconds) +
          ",\"scoring_seconds\":" + num(scoring_seconds) +
          ",\"active_begin_knot\":" + std::to_string(active_begin) +
          ",\"active_end_knot\":" + std::to_string(active_end) +
          ",\"geometry\":" + (cert.safe ? geometry_summary(cert.points, grid) : "null") +
          ",\"failure\":" + (cert.failure.empty() ? "null" : cert.failure);
        if (window_config.window_detailed_diagnostics) {
          candidates += ",\"initial_knots\":" + array(initial) + ",\"final_knots\":" + array(p);
        }
        candidates += '}';
        if (checks >= window_config.window_check_budget) {search_end = "check_budget";break;}
      }
      candidates += ']';
      if (window_config.window_local_refinement && refinement_status == "not_attempted") {
        refinement_status = "search_ended_before_local_pass:" + search_end;
      }
      const auto report_index = reports.size();
      reports.push_back(
        "{\"id\":" + std::to_string(id) + ",\"parent\":" + std::to_string(parent) +
        ",\"depth\":" + std::to_string(depth) + ",\"baseline_begin_s\":" + num(frozen_s[w.a]) +
        ",\"baseline_end_s\":" + num(frozen_s[w.b]) + ",\"result\":\"" +
        (adopted ? "accepted" : "baseline_retained") +
        "\",\"search_end\":\"" + search_end + "\",\"selected_candidate\":" + selected +
        ",\"begin_index\":" + std::to_string(w.a) + ",\"end_index\":" + std::to_string(w.b) +
        ",\"length\":" + num(ss.back()) + ",\"trials\":" + std::to_string(attempts) +
        ",\"geometry\":" + geometry_summary(selected_points, grid) +
        ",\"knots\":" + (window_config.window_detailed_diagnostics ? array(selected_knots) : "[]") +
        ",\"derivative_scales_are_dimensionless\":true,\"baseline_metrics\":" + base_metrics.json() + ",\"knots_reference_s\":" +
        array(adopted ? selected_reference_knots : fitted_knots) +
        ",\"candidates\":" + candidates + ",\"seconds\":" + num(seconds(started)) +
        ",\"fit_seconds\":" + num(fit_seconds) +
        ",\"allocation_seconds\":" + num(allocation) +
        ",\"parent_allocation_seconds\":" + num(parent_budget) +
        ",\"parent_overrun_seconds\":" + num(std::max(0.0, seconds(started) - parent_budget)) +
        ",\"curvature_constraint\":" + num(
          std::min(
            1 / window_config.arc_min_radius,
            original_peak)) +
        ",\"turn_events\":" + std::to_string(window_events) +
        ",\"feature_refinement\":" + (window_config.window_feature_refinement ? "true" : "false") +
        ",\"physical_turns\":" + std::to_string(features.turns.size()) +
        ",\"zero_crossings_s\":" + array(features.zero_crossings) +
        ",\"evaluation_basis\":" + json_quote(
          window_config.window_refine_objective ?
          "common_uniform_arc_samples" : "legacy_mixed") +
        ",\"refinement_status\":" + json_quote(refinement_status) + '}');
      if (!adopted && splittable && checks < window_config.window_check_budget) {
        const auto before_children = geometry_fingerprint(best);
        const double remaining = std::min(
          allocation - seconds(
            started), window_config.window_total_seconds - seconds(begun));
        if (remaining > 0) {
          // Backward ordering preserves the frozen endpoints of pending children.
          optimize({cut, w.b}, depth + 1, id, remaining / 2, checks);
          optimize({w.a, cut}, depth + 1, id, std::max(0.0, allocation - seconds(started)), checks);
        }
        reports[report_index].pop_back();
        reports[report_index] += ",\"child_adoption\":" + std::string(
          before_children != geometry_fingerprint(
            best) ? "true" : "false") + '}';
      }
    };
  const auto groups = windows(frozen, frozen_s, c);
  for (auto it = groups.rbegin(); it != groups.rend(); ++it) {
    std::size_t checks = 0;
    optimize(
      *it, 0, -1,
      std::max(
        0.0, std::min(
          c.window_solver_seconds, c.window_total_seconds - seconds(
            begun))), checks);
  }
  for (std::size_t i = 1; i < best.size(); ++i) {
    best[i - 1].yaw = heading(xy(best[i - 1]), xy(best[i]));
  }
  best.back().yaw = best[best.size() - 2].yaw;
  stats.window_report = "[";
  for (std::size_t i = 0; i < reports.size(); ++i) {
    if (i) {
      stats.window_report += ',';
    }
    stats.window_report += reports[i];
  }
  stats.window_report += ']';stats.geometry_reference_s = array(mapping);
  stats.final_geometry_id = geometry_fingerprint(best);
  return best;
}
}  // namespace plan

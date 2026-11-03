// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "path_experiments.hpp"
#include "path_optimizer.hpp"
#include "bezier_geometry.hpp"
#include "path_geometry_optimizer.hpp"
#include "path_speed_coordination.hpp"

#include <ceres/ceres.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace plan
{
namespace
{
using Clock = std::chrono::steady_clock;
using bezier::add;
using bezier::sub;
using bezier::mul;
using bezier::dot;
using bezier::dist;
using bezier::norm;
using bezier::wrap;
Point2D xy(const PathPoint2D & p) {return {p.x, p.y};}
double elapsed(Clock::time_point t)
{
  return std::chrono::duration<double>(Clock::now() - t).count();
}
Point2D closest(Point2D p, Point2D a, Point2D b)
{
  const auto d = sub(b, a);
  const double n = dot(d, d);
  return add(a, mul(d, n > 0 ? std::clamp(dot(sub(p, a), d) / n, 0.0, 1.0) : 0.0));
}
Point2D nearest(Point2D p, const std::vector<PathPoint2D> & base)
{
  Point2D best = xy(base.front());
  for (std::size_t i = 1; i < base.size(); ++i) {
    const auto q = closest(p, xy(base[i - 1]), xy(base[i]));
    if (dot(sub(p, q), sub(p, q)) < dot(sub(p, best), sub(p, best))) {best = q;}
  }
  return best;
}
std::vector<double> lengths(const std::vector<PathPoint2D> & p)
{
  std::vector<double> s(p.size(), 0.0);
  for (std::size_t i = 1; i < p.size(); ++i) {
    s[i] = s[i - 1] + dist(xy(p[i - 1]), xy(p[i]));
  }
  return s;
}
PathPoint2D at(const std::vector<PathPoint2D> & p, const std::vector<double> & s, double q)
{
  auto it = std::upper_bound(s.begin(), s.end(), q);
  const std::size_t i = it ==
    s.begin() ? 0 : std::min<std::size_t>(it - s.begin() - 1, p.size() - 2);
  const double t = std::clamp((q - s[i]) / (s[i + 1] - s[i]), 0.0, 1.0);
  auto a = p[i]; const auto & b = p[i + 1];
  a.x += (b.x - a.x) * t; a.y += (b.y - a.y) * t;
  a.tangent_yaw += wrap(b.tangent_yaw - a.tangent_yaw) * t;
  a.curvature += (b.curvature - a.curvature) * t;
  return a;
}
void geometry_caps(std::vector<PathPoint2D> & p, const MotionLimits & m)
{
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    auto & a = p[i]; const auto & b = p[i + 1];
    a.curvature_bound = std::max(a.curvature_bound, std::abs(a.curvature));
    if (!b.stop_required) {
      a.curvature_bound = std::max(a.curvature_bound, std::abs(b.curvature));
      a.curvature_rate_bound = std::max(
        a.curvature_rate_bound,
        std::abs(b.curvature - a.curvature) / dist(xy(a), xy(b)));
    }
    a.speed_limit = curve_limits(m, a.curvature_bound, a.curvature_rate_bound).speed;
  }
  p.back().stop_required = true; p.back().speed_limit = 0.0;
}
std::size_t stops(const std::vector<PathPoint2D> & p)
{
  return std::count_if(p.begin(), p.end() - 1, [](const auto & a) {return a.stop_required;});
}
std::size_t speed_events(const SpeedEnvelope & e, const PathProcessingConfig & c)
{
  std::size_t count = 0; int last = 0; double duration = 0, filtered = 0;
  const auto flush = [&]() {if (last && duration >= c.coordination_min_duration) {++count;}};
  for (std::size_t i = 1; i < e.speed.size(); ++i) {
    const double dt = 2 * (e.distance[i] - e.distance[i - 1]) / (e.speed[i] + e.speed[i - 1]);
    const double raw = (e.speed[i] - e.speed[i - 1]) / dt;
    const double alpha = dt / (c.coordination_acceleration_filter_tau + dt);
    filtered = i == 1 ? raw : filtered + alpha * (raw - filtered);
    const double a = filtered;
    const int sign = a > c.coordination_acceleration_threshold ? 1 :
      (a < -c.coordination_acceleration_threshold ? -1 : 0);
    if (sign != last) {flush();duration = 0;last = sign;}
    duration += dt;
  }
  flush(); return count;
}
double roughness(const std::vector<PathPoint2D> & p)
{
  double r = 0;
  for (std::size_t i = 1; i < p.size(); ++i) {
    if (!p[i].stop_required) {
      const double ds = dist(xy(p[i]), xy(p[i - 1]));
      r += std::pow(p[i].curvature - p[i - 1].curvature, 2) / ds;
    }
  }
  return r;
}
auto score(const std::vector<PathPoint2D> & p, const PathProcessingConfig & c)
{
  const auto e = speed_envelope(path_speed_segments(p), c.motion);
  return std::make_tuple(stops(p), speed_events(e, c), roughness(p), executable_time(p, c.motion));
}
std::string geometry_report(const std::vector<PathPoint2D> & p, const PlanningGrid & grid)
{
  double peak = 0, rate = 0, cap = INFINITY, clearance = INFINITY;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    peak = std::max(peak, p[i].curvature_bound);rate = std::max(rate, p[i].curvature_rate_bound);
    cap = std::min(cap, p[i].speed_limit);
    clearance = std::min(clearance, grid.clearance_at(p[i].x, p[i].y));
  }
  std::ostringstream out;out.precision(12);
  out << "{\"peak_curvature_bound\":" << peak << ",\"peak_rate_bound\":" << rate <<
    ",\"minimum_effective_radius\":" <<
    (peak > 0 ? 1 / peak : 0) << ",\"bottleneck_speed\":" << cap <<
    ",\"sampled_minimum_clearance\":" << clearance << ",\"transitions_10_90\":[";
  // Split signed curvature at zero, then measure each monotonic magnitude ramp.
  const auto ss = lengths(p);std::vector<double> s, k;
  for (std::size_t i = 0; i < p.size(); ++i) {
    if (i && p[i - 1].curvature * p[i].curvature < 0) {
      const double t = std::abs(p[i - 1].curvature) /
        (std::abs(p[i - 1].curvature) + std::abs(p[i].curvature));
      s.push_back(ss[i - 1] + t * (ss[i] - ss[i - 1]));k.push_back(0);
    }
    s.push_back(ss[i]);k.push_back(p[i].curvature);
  }
  std::size_t begin = 0;int trend = 0;bool first = true;
  const auto emit = [&](std::size_t end) {
      if (end <= begin) {return;}
      const double lo = std::min(std::abs(k[begin]), std::abs(k[end]));
      const double delta = std::abs(std::abs(k[end]) - std::abs(k[begin]));
      if (delta < 1e-8) {return;}
      double crossings[2]{s[begin], s[end]};
      for (int level = 0; level < 2; ++level) {
        const double target = lo + delta * (level == 0 ? .1 : .9);
        for (std::size_t j = begin + 1; j <= end; ++j) {
          const double a = std::abs(k[j - 1]), b = std::abs(k[j]);
          if (a != b && target >= std::min(a, b) && target <= std::max(a, b)) {
            crossings[level] = s[j - 1] + (target - a) / (b - a) * (s[j] - s[j - 1]);break;
          }
        }
      }
      if (!first) {out << ',';} first = false;
      out << "{\"sign\":" <<
        (k[begin] + k[end] >= 0 ? 1 : -1) << ",\"curvature_change\":" << delta <<
        ",\"distance\":" << std::abs(crossings[1] - crossings[0]) << "}";
    };
  for (std::size_t i = 1; i < k.size(); ++i) {
    const double delta = std::abs(k[i]) - std::abs(k[i - 1]);
    const int next = delta > 1e-10 ? 1 : delta < -1e-10 ? -1 : 0;
    if ((next && trend && next != trend) || (i > 1 && k[i - 1] == 0)) {
      emit(i - 1);begin = i - 1;trend = 0;
    }
    if (next) {trend = next;}
  }
  emit(k.size() - 1);out << "]}";return out.str();
}
struct Window {std::size_t begin, end;};
std::vector<Window> windows(const std::vector<PathPoint2D> & p, const PathProcessingConfig & c)
{
  const auto s = lengths(p);
  std::vector<Window> events;
  for (std::size_t i = 0; i + 1 < p.size(); ++i) {
    if (p[i].curvature_bound <= 1e-10 && !p[i].stop_required) {continue;}
    if (!events.empty() && i <= events.back().end) {events.back().end = i + 1;} else {
      events.push_back({i, i + 1});
    }
  }
  std::vector<Window> merged;
  for (auto w:events) {
    if (!merged.empty() && s[w.begin] - s[merged.back().end] <= c.window_merge_gap) {
      merged.back().end = w.end;
    } else {merged.push_back(w);}
  }
  std::vector<Window> out;
  for (auto w:merged) {
    const double a = s[w.begin], b = s[w.end];
    while (w.begin > 0 && a - s[w.begin - 1] <= c.window_extension) {--w.begin;}
    while (w.end + 1 < p.size() && s[w.end + 1] - b <= c.window_extension) {++w.end;}
    if (!out.empty() && w.begin <= out.back().end) {out.back().end = w.end;} else {
      out.push_back(w);
    }
  }
  return out;
}

void coordinate_legacy_speed(
  std::vector<PathPoint2D> & best, SpeedEnvelope & envelope,
  const PathProcessingConfig & c, double time_limit)
{
  if (!c.enable_turn_speed_coordination) {return;}
  if (c.coordination_enable_platforms) {
    const auto s = lengths(best);
    for (const auto w:windows(best, c)) {
      std::size_t begin = w.begin, end = w.end;
      while (begin < end && best[begin].curvature_bound <= 1e-10) {++begin;}
      while (end > begin && best[end - 1].curvature_bound <= 1e-10) {--end;}
      if (end <= begin || s[end] - s[begin] < c.coordination_platform_min_length ||
        std::any_of(
          best.begin() + begin, best.begin() + end, [](const auto & p) {
            return p.stop_required;
          })) {continue;}
      double cap = c.motion.v_nominal, peak = 0;
      for (std::size_t i = begin; i < end; ++i) {
        cap = std::min(cap, best[i].speed_limit);
        peak = std::max(peak, std::min(c.motion.v_nominal, best[i].speed_limit));
      }
      if (cap < c.coordination_platform_min_speed || peak - cap < c.coordination_min_gain) {
        continue;
      }
      auto trial = best;
      for (std::size_t i = begin; i < end; ++i) {
        trial[i].speed_limit = std::min(trial[i].speed_limit, cap);
      }
      auto candidate = speed_envelope(path_speed_segments(trial), c.motion);
      if (speed_events(
          candidate,
          c) <= speed_events(envelope, c) && executable_time(trial, c.motion) <= time_limit)
      {
        best = std::move(trial);envelope = std::move(candidate);
      }
    }
  }
  for (std::size_t round = 0; round < c.coordination_max_rounds; ++round) {
    const auto initial_events = speed_events(envelope, c);
    struct Merge {std::size_t a, b;double cap, priority;};
    std::vector<std::size_t> minima;
    for (std::size_t i = 1; i + 1 < best.size(); ++i) {
      if (!best[i].stop_required && best[i].speed_limit <= best[i - 1].speed_limit &&
        best[i].speed_limit < best[i + 1].speed_limit) {minima.push_back(i);}
    }
    const auto s = lengths(best);std::vector<Merge> merges;
    for (std::size_t i = 1; i < minima.size(); ++i) {
      const auto a = minima[i - 1], b = minima[i];
      if (s[b] - s[a] > c.coordination_max_gap ||
        std::any_of(
          best.begin() + a, best.begin() + b + 1, [](const auto & p) {
            return p.stop_required;
          })) {continue;}
      const double cap = std::max(best[a].speed_limit, best[b].speed_limit);
      double peak = cap;
      for (std::size_t j = a; j <= b; ++j) {
        peak = std::max(peak, best[j].speed_limit);
      }
      if (peak - cap < c.coordination_min_gain) {continue;}
      auto trial = best;
      for (std::size_t j = a; j <= b; ++j) {
        trial[j].speed_limit = std::min(trial[j].speed_limit, cap);
      }
      const auto e = speed_envelope(path_speed_segments(trial), c.motion);
      const auto before = speed_events(envelope, c), after = speed_events(e, c);
      if (after < before) {
        merges.push_back({a, b, cap, (before - after) / std::max(1e-9, e.time - envelope.time)});
      }
    }
    std::stable_sort(
      merges.begin(), merges.end(), [](auto a, auto b) {
        return a.priority > b.priority;
      });
    for (const auto & m:merges) {
      auto trial = best;
      for (std::size_t j = m.a; j <= m.b; ++j) {
        trial[j].speed_limit = std::min(trial[j].speed_limit, m.cap);
      }
      auto e = speed_envelope(path_speed_segments(trial), c.motion);
      if (speed_events(
          e,
          c) <
        speed_events(
          envelope,
          c) && executable_time(trial, c.motion) <= time_limit)
      {
        best = std::move(trial);envelope = std::move(e);
      }
    }

    if (speed_events(envelope, c) >= initial_events) {break;}
  }
}
void coordinate_speed(
  std::vector<PathPoint2D> & path, SpeedEnvelope & envelope,
  const PathProcessingConfig & c, double time_limit, std::string * report = nullptr)
{
  if (!c.enable_turn_speed_coordination) {
    if (report) {*report = "{\"policy\":\"disabled\"}";}
    return;
  }
  if (c.coordination_policy == "bounded") {
    std::string trace;
    coordinate_bounded_speed(path, envelope, c, time_limit, trace);
    if (report) {*report = std::move(trace);}
  } else {
    coordinate_legacy_speed(path, envelope, c, time_limit);
    if (report) {*report = "{\"policy\":\"legacy\"}";}
  }
}
struct Objective
{
  const PlanningGrid & grid;
  const std::vector<PathPoint2D> & baseline;
  const PathProcessingConfig & c;
  std::size_t knots;
  bool operator()(double const * const * parameters, double * r) const
  {
    const double * p = parameters[0]; std::size_t index = 0;
    const double kn = 1 / c.window_target_radius, qn = kn / c.window_transition_scale;
    double last_rate = 0;
    for (std::size_t i = 1; i < knots; ++i) {
      const auto b = bezier::from_knots(p + 5 * (i - 1), p + 5 * i);
      const double weight = std::sqrt(
        std::max(1e-6, dist(b.front(), b.back())) /
        (c.window_objective_samples + 1));
      for (std::size_t j = 0; j <= c.window_objective_samples; ++j) {
        const auto g = bezier::geometry(b, static_cast<double>(j) / c.window_objective_samples);
        const auto ref = nearest(g.point, baseline);
        const double v = curve_limits(c.motion, std::abs(g.curvature), std::abs(g.rate)).speed;
        r[index++] = weight * std::sqrt(c.window_curvature_weight) * g.curvature / kn;
        r[index++] = weight * std::sqrt(c.window_rate_weight) * g.rate / qn;
        r[index++] = weight * std::sqrt(c.window_deviation_weight) * (g.point.x - ref.x) /
          c.window_max_deviation;
        r[index++] = weight * std::sqrt(c.window_deviation_weight) * (g.point.y - ref.y) /
          c.window_max_deviation;
        r[index++] = weight * std::sqrt(c.window_clearance_weight) *
          std::max(0.0, 1 - grid.clearance_at(g.point.x, g.point.y) / c.clearance_target);
        r[index++] = weight * std::sqrt(c.window_speed_weight) *
          std::max(0.0, 1 - v / c.window_target_speed);
        r[index++] = weight * std::sqrt(c.window_curvature_weight) *
          std::max(0.0, std::abs(g.curvature) / kn - 1);
        r[index++] = weight * std::sqrt(c.window_constraint_weight) *
          std::max(0.0, std::abs(g.rate) / c.max_curvature_rate - c.window_constraint_fraction);
        r[index++] = weight * std::sqrt(c.window_constraint_weight) *
          std::max(0.0, std::abs(g.curvature) * c.arc_min_radius - c.window_constraint_fraction);
        if (!std::isfinite(g.curvature) || !std::isfinite(g.rate)) {return false;}
      }
      for (std::size_t j = 0; j < c.window_objective_samples; ++j) {
        const auto leaf = bezier::interval(
          b, static_cast<double>(j) / c.window_objective_samples,
          static_cast<double>(j + 1) / c.window_objective_samples);
        const double chord = std::max(1e-10, dist(leaf.front(), leaf.back()));
        const auto bound = bezier::bounds(leaf, chord * c.window_min_tangent_ratio);
        const double hard = weight * std::sqrt(c.window_constraint_weight);
        r[index++] = hard * std::max(
          0.0,
          bound.curvature * c.arc_min_radius -
          c.window_constraint_fraction);
        r[index++] = hard * std::max(
          0.0,
          bound.rate / c.max_curvature_rate -
          c.window_constraint_fraction);
        r[index++] = hard * std::max(0.0, c.window_min_tangent_ratio - bound.lower / chord);
      }
      const double first = bezier::geometry(b, 0).rate;
      r[index++] = i > 1 ? std::sqrt(c.window_join_weight) * (first - last_rate) / qn : 0;
      last_rate = bezier::geometry(b, 1).rate;
    }
    return true;
  }
};
struct SegmentObjective
{
  const PlanningGrid & grid;
  const std::vector<PathPoint2D> & baseline;
  const PathProcessingConfig & config;
  bool operator()(double const * const * parameters, double * residuals) const
  {
    double p[10];
    std::copy(parameters[0], parameters[0] + 5, p);
    std::copy(parameters[1], parameters[1] + 5, p + 5);
    const double * blocks[]{p};
    return Objective{grid, baseline, config, 2}(blocks, residuals);
  }
};
struct JoinObjective
{
  double scale;
  bool operator()(double const * const * p, double * r) const
  {
    r[0] = scale * (bezier::geometry(bezier::from_knots(p[0], p[1]), 1).rate -
      bezier::geometry(bezier::from_knots(p[1], p[2]), 0).rate);
    return std::isfinite(r[0]);
  }
};
struct Certificate
{
  std::size_t used = 0;
  std::string reason = "geometry";
  double rejected_curvature = 0.0, rejected_rate = 0.0;
  std::vector<PathPoint2D> points;
  std::vector<double> envelopes;
};
bool certify(
  const bezier::Bezier & b, const PlanningGrid & grid,
  const std::vector<PathPoint2D> & reference, const PathProcessingConfig & c,
  std::size_t depth, Certificate & cert)
{
  if (++cert.used > c.window_check_budget) {cert.reason = "check_budget";return false;}
  const auto bounds = bezier::bounds(b);
  const auto a = bezier::geometry(b, 0), z = bezier::geometry(b, 1);
  const auto middle = bezier::geometry(b, .5);
  const double sampled_peak =
    std::max({std::abs(a.curvature), std::abs(z.curvature), std::abs(middle.curvature)});
  const double sampled_rate = std::max({std::abs(a.rate), std::abs(z.rate), std::abs(middle.rate)});
  if (sampled_peak > 1 / c.arc_min_radius || sampled_rate > c.max_curvature_rate) {
    cert.rejected_curvature = sampled_peak;cert.rejected_rate = sampled_rate;
    cert.reason = "sampled_curvature_or_rate";
    return false;
  }
  const auto ref = nearest(bezier::geometry(b, .5).point, reference);
  double deviation = 0;
  for (const auto & p:b) {
    deviation = std::max(deviation, dist(p, ref));
  }
  const double chord = dist(b.front(), b.back());
  const double rate = std::max(
    bounds.rate, chord > 0 ? std::abs(
      z.curvature - a.curvature) / chord : INFINITY);
  const double peak = std::max({bounds.curvature, std::abs(a.curvature), std::abs(z.curvature)});
  const bool regular = bounds.lower > 1e-10 && peak <= 1 / c.arc_min_radius &&
    rate <= c.max_curvature_rate;
  const bool small = chord <= c.resample_spacing && bounds.envelope <= c.arc_max_deviation &&
    std::abs(wrap(z.yaw - a.yaw)) <= c.arc_max_angle &&
    std::abs(
    wrap(
      a.yaw -
      std::atan2(b.back().y - b.front().y, b.back().x - b.front().x))) <= c.arc_max_angle &&
    std::abs(
    wrap(
      z.yaw -
      std::atan2(b.back().y - b.front().y, b.back().x - b.front().x))) <= c.arc_max_angle;
  const bool feasible = regular &&
    curve_limits(c.motion, peak, rate).speed >= c.motion.min_turn_speed;
  if (!small || !feasible || deviation > c.window_max_deviation) {
    if (depth >= c.curve_max_depth) {
      cert.rejected_curvature = peak;
      cert.rejected_rate = rate;
      cert.reason =
        !regular ? "curvature_or_regularity" : !small ? "sampling" : "deviation_or_speed";
      return false;
    }
    const auto halves = bezier::split(b);
    return certify(halves.first, grid, reference, c, depth + 1, cert) &&
           certify(halves.second, grid, reference, c, depth + 1, cert);
  }
  if (!curve_chord_is_free(grid, b.front(), b.back(), bounds.envelope + 1e-10)) {
    cert.reason = "collision";return false;
  }
  if (cert.points.size() + 1 >= c.max_output_points) {cert.reason = "point_limit";return false;}
  PathPoint2D p{a.point.x, a.point.y, a.yaw, a.yaw, a.curvature, false};
  p.segment_type = 1;p.curvature_bound = peak;p.curvature_rate_bound = rate;
  p.speed_limit = curve_limits(c.motion, peak, rate).speed;
  if (!cert.points.empty()) {cert.points.back() = p;} else {cert.points.push_back(p);}
  cert.envelopes.push_back(bounds.envelope);
  p.x = z.point.x;p.y = z.point.y;p.yaw = p.tangent_yaw = z.yaw;p.curvature = z.curvature;
  p.curvature_bound = std::abs(z.curvature);p.curvature_rate_bound = 0;
  cert.points.push_back(p);
  return true;
}
double segment_distance(Point2D a, Point2D b, Point2D c, Point2D d)
{
  const auto u = sub(b, a), v = sub(d, c);
  const double denom = bezier::cross(u, v);
  if (std::abs(denom) > 1e-12) {
    const double t = bezier::cross(sub(c, a), v) / denom;
    const double q = bezier::cross(sub(c, a), u) / denom;
    if (t >= 0 && t <= 1 && q >= 0 && q <= 1) {return 0;}
  }
  return std::min(
    {dist(a, closest(a, c, d)), dist(b, closest(b, c, d)),
      dist(c, closest(c, a, b)), dist(d, closest(d, a, b))});
}
bool self_free(const Certificate & c)
{
  for (std::size_t i = 0; i < c.envelopes.size(); ++i) {
    for (std::size_t j = i + 2; j < c.envelopes.size(); ++j) {
      if (segment_distance(
          xy(c.points[i]), xy(c.points[i + 1]), xy(c.points[j]),
          xy(c.points[j + 1])) <=
        c.envelopes[i] + c.envelopes[j] + 1e-10) {return false;}
    }
  }
  return true;
}
std::vector<PathPoint2D> splice(
  const std::vector<PathPoint2D> & p, Window w, std::vector<PathPoint2D> curve)
{
  curve.back() = p[w.end];  // preserve the outgoing certificate and any boundary stop
  std::vector<PathPoint2D> out(p.begin(), p.begin() + w.begin);
  out.insert(out.end(), curve.begin(), curve.end());
  out.insert(out.end(), p.begin() + w.end + 1, p.end());
  for (std::size_t i = 0; i + 1 < out.size(); ++i) {
    out[i].yaw = std::atan2(out[i + 1].y - out[i].y, out[i + 1].x - out[i].x);
  }
  out.back().yaw = out[out.size() - 2].yaw;
  return out;
}
bool global_free(const PlanningGrid & grid, const std::vector<PathPoint2D> & p)
{
  for (std::size_t i = 1; i < p.size(); ++i) {
    if (dist(xy(p[i - 1]), xy(p[i])) <= 1e-10 || !line_of_sight(grid, xy(p[i - 1]), xy(p[i]))) {
      return false;
    }
  }
  return true;
}
}  // namespace

std::vector<SpeedSegment> path_speed_segments(const std::vector<PathPoint2D> & path)
{
  std::vector<SpeedSegment> edges;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto & a = path[i - 1]; const auto & b = path[i];
    edges.push_back(
      {dist(xy(a), xy(b)), a.curvature_bound, a.curvature_rate_bound,
        a.speed_limit, b.stop_required});
  }
  return edges;
}
std::size_t path_speed_events(
  const std::vector<PathPoint2D> & path,
  const PathProcessingConfig & config)
{
  return speed_events(speed_envelope(path_speed_segments(path), config.motion), config);
}
std::size_t speed_envelope_events(
  const SpeedEnvelope & envelope,
  const PathProcessingConfig & config)
{
  return speed_events(envelope, config);
}
double executable_time(const std::vector<PathPoint2D> & p, const MotionLimits & m)
{
  if (p.size() < 2) {return 0;}
  double t = speed_envelope(path_speed_segments(p), m).time;
  for (std::size_t i = 1; i + 1 < p.size(); ++i) {
    if (p[i].stop_required) {
      const double incoming = std::atan2(p[i].y - p[i - 1].y, p[i].x - p[i - 1].x);
      t += std::abs(wrap(p[i].tangent_yaw - incoming)) / m.w_max;
    }
  }
  return t;
}

std::vector<PathPoint2D> process_frozen_speed(
  const std::vector<PathPoint2D> & path, const PathProcessingConfig & c,
  PathProcessingStats & stats, double time_limit)
{
  validate_path_processing_config(c);
  if (path.empty()) {return path;}
  if (!path.back().stop_required) {
    throw std::invalid_argument("Frozen geometry lacks terminal stop");
  }
  for (const auto & p:path) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
      !std::isfinite(p.tangent_yaw) || !std::isfinite(p.curvature) ||
      !std::isfinite(p.curvature_bound) || !std::isfinite(p.curvature_rate_bound) ||
      p.curvature_bound < 0 || p.curvature_rate_bound < 0)
    {
      throw std::invalid_argument("Invalid frozen geometry metadata");
    }
  }
  auto base = path;
  for (std::size_t i = 0; i + 1 < base.size(); ++i) {
    const auto & a = base[i];const auto & b = base[i + 1];
    const double ds = dist(xy(a), xy(b));
    if (!std::isfinite(ds) || ds <= 0) {throw std::invalid_argument("Invalid frozen segment");}
    const double k = b.stop_required ? std::abs(a.curvature) :
      std::max(std::abs(a.curvature), std::abs(b.curvature));
    const double q = b.stop_required ? 0 : std::abs(b.curvature - a.curvature) / ds;
    if (a.curvature_bound + 1e-8 < k || a.curvature_rate_bound + 1e-8 < q) {
      throw std::invalid_argument("Frozen certificates do not cover nodal geometry");
    }
    base[i].speed_limit = curve_limits(c.motion, a.curvature_bound, a.curvature_rate_bound).speed;
  }
  base.back().speed_limit = 0;
  auto envelope = speed_envelope(path_speed_segments(base), c.motion);
  const auto original = envelope;
  auto best = base;
  stats.speed_events_before = speed_events(original, c);
  try {
    coordinate_speed(best, envelope, c, time_limit, &stats.coordination_report);
    if (executable_time(best, c.motion) > time_limit + 1e-8) {
      best = base;envelope = original;
      stats.coordination_report = "{\"fallback\":\"time_limit\",\"attempt\":" +
        stats.coordination_report + '}';
    }
  } catch (const std::runtime_error & error) {
    best = base;envelope = original;
    std::ostringstream out;
    out << "{\"fallback\":\"speed_solver\",\"error\":" << std::quoted(error.what()) << '}';
    stats.coordination_report = out.str();
  }
  stats.dynamic_budget = c.motion.enable_dynamic_angular_budget;
  stats.speed_events_after = speed_events(envelope, c);
  stats.speed_envelope_fallback = envelope.conservative_fallback;
  stats.estimated_time = executable_time(best, c.motion);
  stats.selected_length = lengths(best).back();stats.retained_corners = stops(best);
  stats.final_geometry_id = geometry_fingerprint(best);
  if (stats.geometry_policy.empty()) {stats.geometry_policy = "frozen_replay";}
  return best;
}

std::vector<PathPoint2D> process_experiments(
  const PlanningGrid & grid, const std::vector<PathPoint2D> & baseline,
  const PathProcessingConfig & c, PathProcessingStats & stats)
{
  if (baseline.empty()) {return baseline;}
  if (baseline.size() == 1) {
    auto point = baseline.front();
    point.stop_required = true;point.speed_limit = 0.0;
    stats.dynamic_budget = c.motion.enable_dynamic_angular_budget;
    stats.window_report = "[]";
    return {point};
  }
  if (c.enable_window_bezier && c.window_geometry_policy == "geometry_first") {
    auto best = optimize_window_geometry(grid, baseline, c, stats);
    geometry_caps(best, c.motion);
    stats.dynamic_budget = c.motion.enable_dynamic_angular_budget;
    if (executable_time(best, c.motion) > stats.frozen_baseline_time * c.window_max_time_ratio ||
      lengths(best).back() > stats.baseline_length * c.max_length_ratio || !global_free(grid, best))
    {
      best = baseline;geometry_caps(best, c.motion);
      stats.geometry_fallback_reason = "final_length_time_or_collision";
      std::ostringstream mapping;mapping.precision(16);mapping << '[';
      const auto s = lengths(best);
      for (std::size_t i = 0; i < s.size(); ++i) {
        if (i) {
          mapping << ',';
        }
        mapping << s[i];
      }
      mapping << ']';stats.geometry_reference_s = mapping.str();
    }
    best =
      process_frozen_speed(best, c, stats, stats.frozen_baseline_time * c.window_max_time_ratio);
    stats.selected_length = lengths(best).back();stats.estimated_time = executable_time(
      best,
      c.motion);
    stats.retained_corners = stops(best);stats.minimum_clearance = INFINITY;
    for (const auto & p:best) {
      stats.minimum_clearance = std::min(stats.minimum_clearance, grid.clearance_at(p.x, p.y));
    }
    stats.final_geometry_id = geometry_fingerprint(best);
    return best;
  }
  auto old = c.motion;old.enable_dynamic_angular_budget = false;
  auto frozen = baseline;geometry_caps(frozen, old);
  auto baseline_config = c;baseline_config.motion = old;
  const double l0 = lengths(frozen).back(), t0 = legacy_executable_time(frozen, baseline_config);
  auto best = frozen;geometry_caps(best, c.motion);
  stats.baseline_length = l0;stats.frozen_baseline_time = t0;
  stats.dynamic_budget = c.motion.enable_dynamic_angular_budget;
  const auto original_windows = windows(best, c);
  std::ostringstream report; report.precision(12);report << '[';
  const auto started = Clock::now();
  // Process backward so earlier frozen indices remain valid after a replacement.
  bool first_report = true;
  if (c.enable_window_bezier) {
    for (auto wi = original_windows.rbegin(); wi != original_windows.rend(); ++wi) {
      const Window w = *wi;
      std::string reason = "no_improvement";std::size_t trials = 0, checks = 0;
      double rejected_curvature = 0, rejected_rate = 0, initial_cost = 0, final_cost = 0;
      std::size_t iterations = 0;
      std::size_t candidate_events = 0;double candidate_roughness = 0, candidate_time = 0;
      const std::vector<PathPoint2D> reference(best.begin() + w.begin, best.begin() + w.end + 1);
      const auto s = lengths(reference);const double total = s.back();
      std::vector<double> anchors{0.0};
      for (std::size_t i = 1; i + 1 < reference.size(); ++i) {
        const bool event = reference[i].stop_required ||
          (reference[i - 1].curvature_bound <= 1e-10 && reference[i].curvature_bound > 1e-10);
        if (event && s[i] - anchors.back() >= c.window_knot_spacing / 2 &&
          total - s[i] >= c.window_knot_spacing / 2) {anchors.push_back(s[i]);}
      }
      anchors.push_back(total);
      std::vector<double> knots{0.0};
      // Uniformly subdivide each event interval; never leave a tiny last segment.
      for (std::size_t i = 1; i < anchors.size(); ++i) {
        const auto count =
          static_cast<std::size_t>(std::ceil(
            (anchors[i] - anchors[i - 1]) / c.window_knot_spacing));
        for (std::size_t j = 1; j <= count; ++j) {
          knots.push_back(anchors[i - 1] + (anchors[i] - anchors[i - 1]) * j / count);
        }
      }
      const auto window_started = Clock::now();
      std::vector<PathPoint2D> chosen;
      std::vector<double> chosen_knots;std::string chosen_geometry;
      auto winning_score = score(best, c);
      for (double scale:c.window_seed_scales) {
        if (elapsed(started) >= c.window_total_seconds ||
          elapsed(window_started) >= c.window_solver_seconds)
        {
          if (reason == "no_improvement") {
            reason = "solver_budget";
          }
          break;
        }
        if (knots.size() > c.window_max_knots) {reason = "knot_limit";break;}
        std::vector<double> p;
        for (std::size_t j = 0; j < knots.size(); ++j) {
          const auto a = at(reference, s, knots[j]);
          const double h = j == 0 ? knots[1] : j + 1 ==
            knots.size() ? total - knots[j - 1] : (knots[j + 1] - knots[j - 1]) / 2;
          p.insert(p.end(), {a.x, a.y, a.tangent_yaw, h * scale, a.curvature});
        }
        ceres::Problem problem;
        for (std::size_t j = 1; j < knots.size(); ++j) {
          auto * cost = new ceres::DynamicNumericDiffCostFunction<SegmentObjective, ceres::CENTRAL>(
            new SegmentObjective{grid, reference, c});
          cost->AddParameterBlock(5);cost->AddParameterBlock(5);
          cost->SetNumResiduals(
            static_cast<int>(9 * (c.window_objective_samples + 1) + 3 *
            c.window_objective_samples + 1));
          problem.AddResidualBlock(cost, nullptr, p.data() + 5 * (j - 1), p.data() + 5 * j);
          if (j > 1) {
            auto * join = new ceres::DynamicNumericDiffCostFunction<JoinObjective, ceres::CENTRAL>(
              new JoinObjective{std::sqrt(
                  c.window_join_weight) * c.window_target_radius * c.window_transition_scale});
            for (int k = 0; k < 3; ++k) {
              join->AddParameterBlock(5);
            }
            join->SetNumResiduals(1);
            problem.AddResidualBlock(
              join, nullptr, p.data() + 5 * (j - 2),
              p.data() + 5 * (j - 1), p.data() + 5 * j);
          }
        }
        const auto seed = p;
        for (std::size_t j = 0; j < knots.size(); ++j) {
          double * block = p.data() + 5 * j;
          if (j == 0 || j + 1 == knots.size()) {
            problem.SetParameterization(block, new ceres::SubsetParameterization(5, {0, 1, 2, 4}));
          }
          for (int k:{0, 1}) {
            problem.SetParameterLowerBound(
              block, k, seed[5 * j + k] - c.window_max_deviation / std::sqrt(
                2.0));
            problem.SetParameterUpperBound(
              block, k, seed[5 * j + k] + c.window_max_deviation / std::sqrt(
                2.0));
          }
          problem.SetParameterLowerBound(block, 2, seed[5 * j + 2] - c.window_heading_max_change);
          problem.SetParameterUpperBound(block, 2, seed[5 * j + 2] + c.window_heading_max_change);
          problem.SetParameterLowerBound(block, 3, seed[5 * j + 3] * c.window_derivative_min_scale);
          problem.SetParameterUpperBound(block, 3, seed[5 * j + 3] * c.window_derivative_max_scale);
          problem.SetParameterLowerBound(block, 4, -1 / c.arc_min_radius);
          problem.SetParameterUpperBound(block, 4, 1 / c.arc_min_radius);
        }
        ceres::Solver::Options options;
        options.max_num_iterations = static_cast<int>(c.window_iterations);
        options.num_threads = 1;options.linear_solver_type = ceres::DENSE_QR;
        options.max_solver_time_in_seconds = std::max(
          1e-6, std::min(
            c.window_solver_seconds - elapsed(window_started),
            c.window_total_seconds - elapsed(started)));
        ceres::Solver::Summary summary;ceres::Solve(options, &problem, &summary);++trials;
        initial_cost = summary.initial_cost;final_cost = summary.final_cost;
        iterations = summary.iterations.size();
        if (!summary.IsSolutionUsable()) {reason = "solver_failure";continue;}
        Certificate cert;cert.used = checks;
        for (std::size_t j = 1; j < knots.size(); ++j) {
          if (!certify(
              bezier::from_knots(p.data() + 5 * (j - 1), p.data() + 5 * j), grid,
              reference, c, 0, cert)) {break;}
          if (j + 1 == knots.size()) {cert.reason = "certified";}
        }
        checks = cert.used;rejected_curvature = cert.rejected_curvature;
        rejected_rate = cert.rejected_rate;
        if (cert.reason != "certified") {reason = cert.reason;continue;}
        if (!self_free(cert)) {reason = "self_intersection";continue;}
        // Reject crossings with the unchanged path, using the curve envelope.
        bool crossing = false;
        for (std::size_t i = 0; i < cert.envelopes.size() && !crossing; ++i) {
          for (std::size_t j = 0; j + 1 < best.size(); ++j) {
            if ((j >= w.begin && j < w.end) || (j + 1 == w.begin && i == 0) ||
              (j == w.end && i + 1 == cert.envelopes.size())) {continue;}
            if (segment_distance(
                xy(cert.points[i]), xy(cert.points[i + 1]), xy(best[j]),
                xy(best[j + 1])) <= cert.envelopes[i] + 1e-10) {crossing = true;break;}
          }
        }
        if (crossing) {reason = "outside_intersection";continue;}
        const auto geometry = geometry_report(cert.points, grid);
        auto candidate = splice(best, w, std::move(cert.points));
        if (candidate.size() > c.max_output_points || !global_free(grid, candidate)) {
          reason = "final_collision_or_size";continue;
        }
        if (lengths(candidate).back() > l0 * c.max_length_ratio ||
          executable_time(candidate, c.motion) > t0 * c.window_max_time_ratio)
        {
          reason = "length_or_time";continue;
        }
        auto candidate_envelope = speed_envelope(path_speed_segments(candidate), c.motion);
        coordinate_speed(candidate, candidate_envelope, c, t0 * c.window_max_time_ratio);
        const auto value = score(candidate, c);
        candidate_events = std::get<1>(value);candidate_roughness = std::get<2>(value);
        candidate_time = std::get<3>(value);
        if (value < winning_score) {
          chosen = std::move(candidate);chosen_knots = p;chosen_geometry = geometry;
          winning_score = value;reason = "accepted";
        }
      }
      if (!chosen.empty()) {best = std::move(chosen);reason = "accepted";}
      if (chosen_geometry.empty()) {chosen_geometry = geometry_report(reference, grid);}
      if (!first_report) {report << ',';} first_report = false;
      report << "{\"geometry\":" << chosen_geometry << ",\"knots\":[";
      for (std::size_t i = 0; i < chosen_knots.size(); ++i) {
        if (i) {
          report << ',';
        }
        report << chosen_knots[i];
      }
      report << "],\"baseline_begin_s\":" << lengths(frozen)[w.begin] <<
        ",\"baseline_end_s\":" << lengths(frozen)[w.end] << ",\"begin_index\":" << w.begin <<
        ",\"end_index\":" << w.end <<
        ",\"length\":" << total << ",\"trials\":" << trials << ",\"checks\":" << checks <<
        ",\"candidate_events\":" << candidate_events << ",\"candidate_roughness\":" <<
        candidate_roughness <<
        ",\"candidate_time\":" << candidate_time << ",\"iterations\":" << iterations <<
        ",\"initial_cost\":" << initial_cost <<
        ",\"final_cost\":" << final_cost << ",\"rejected_curvature\":" << rejected_curvature <<
        ",\"rejected_rate\":" << (std::isfinite(rejected_rate) ? rejected_rate : -1.0) <<
        ",\"seconds\":" << elapsed(window_started) << ",\"result\":\"" << reason << "\"}";
    }
  }
  report << ']';stats.window_report = report.str();
  auto envelope = speed_envelope(path_speed_segments(best), c.motion);
  stats.speed_events_before = speed_events(envelope, c);
  coordinate_speed(best, envelope, c, t0 * c.window_max_time_ratio);
  if (executable_time(best, c.motion) > t0 * c.window_max_time_ratio ||
    lengths(best).back() > l0 * c.max_length_ratio || !global_free(grid, best))
  {
    best = frozen;geometry_caps(best, c.motion);
    stats.window_report = "[{\"result\":\"global_fallback\"}]";
  }
  envelope = speed_envelope(path_speed_segments(best), c.motion);
  stats.speed_events_after = speed_events(envelope, c);
  stats.speed_envelope_fallback = envelope.conservative_fallback;
  stats.selected_length = lengths(best).back();
  stats.estimated_time = executable_time(best, c.motion);
  stats.retained_corners = stops(best);
  stats.minimum_clearance = INFINITY;
  for (const auto & p:best) {
    stats.minimum_clearance = std::min(stats.minimum_clearance, grid.clearance_at(p.x, p.y));
  }
  return best;
}
}  // namespace plan

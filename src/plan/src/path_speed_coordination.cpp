// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "path_speed_coordination.hpp"
#include "path_experiments.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <tuple>
#include <utility>

namespace plan
{
double envelope_interval_time(const SpeedEnvelope & envelope, double begin, double end)
{
  double time = 0;
  for (std::size_t i = 1; i < envelope.distance.size(); ++i) {
    const double a = std::max(begin, envelope.distance[i - 1]);
    const double b = std::min(end, envelope.distance[i]);
    if (b <= a) {continue;}
    const double ds = envelope.distance[i] - envelope.distance[i - 1];
    const double v2 = envelope.speed[i - 1] * envelope.speed[i - 1];
    const double slope = (envelope.speed[i] * envelope.speed[i] - v2) / ds;
    const double va = std::sqrt(std::max(0.0, v2 + slope * (a - envelope.distance[i - 1])));
    const double vb = std::sqrt(std::max(0.0, v2 + slope * (b - envelope.distance[i - 1])));
    if (va + vb <= 0) {return std::numeric_limits<double>::infinity();}
    time += 2 * (b - a) / (va + vb);
  }
  return time;
}
double envelope_variation(const SpeedEnvelope & envelope)
{
  double total = 0;
  for (std::size_t i = 1; i < envelope.speed.size(); ++i) {
    total += std::abs(envelope.speed[i] - envelope.speed[i - 1]);
  }
  return total;
}
namespace
{
using Span = std::pair<double, double>;
std::vector<std::size_t> valleys(const SpeedEnvelope & e)
{
  std::vector<std::size_t> out;int previous = 0;
  for (std::size_t i = 1; i < e.speed.size(); ++i) {
    const double delta = e.speed[i] - e.speed[i - 1];
    const int trend = delta > 1e-10 ? 1 : delta < -1e-10 ? -1 : 0;
    if (trend > 0 && previous < 0 && e.speed[i - 1] > 1e-10) {out.push_back(i - 1);}
    if (trend) {previous = trend;}
  }
  return out;
}
bool local_time_ok(
  const SpeedEnvelope & original, const SpeedEnvelope & trial, Span span,
  double loss)
{
  return envelope_interval_time(trial, span.first, span.second) <=
         envelope_interval_time(original, span.first, span.second) * (1 + loss) + 1e-8;
}
std::vector<Span> turn_groups(
  const std::vector<PathPoint2D> & path, const PathProcessingConfig & c)
{
  std::vector<Span> groups;double s = 0;bool stopped = false;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    if (path[i].stop_required) {stopped = true;}
    const double end = s + std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
    const bool curved = path[i].curvature_bound >= c.coordination_curvature_threshold ||
      curve_limits(c.motion, path[i].curvature_bound, path[i].curvature_rate_bound).speed <
      c.motion.v_nominal - 1e-10;
    if (curved) {
      if (!groups.empty() && !stopped && s - groups.back().second <= c.coordination_group_max_gap) {
        groups.back().second = end;
      } else {groups.push_back({s, end});}
      stopped = false;
    }
    if (path[i + 1].stop_required) {stopped = true;}
    s = end;
  }
  return groups;
}
}  // namespace

void coordinate_bounded_speed(
  std::vector<PathPoint2D> & path, SpeedEnvelope & envelope, const PathProcessingConfig & c,
  double time_limit, std::string & report)
{
  const auto original = envelope;
  const double alignment_time = executable_time(path, c.motion) - original.time;
  const auto groups = turn_groups(path, c);
  std::vector<std::size_t> selected;
  std::ostringstream trials;trials.precision(12);trials << '[';
  std::size_t attempted = 0;
  for (std::size_t round = 0; round < c.coordination_max_rounds; ++round) {
    const auto minima = valleys(envelope);
    const auto before_events = speed_envelope_events(envelope, c);
    const double before_variation = envelope_variation(envelope);
    auto winning_score = std::make_tuple(before_events, before_variation, envelope.time);
    auto winning_path = path;auto winning_envelope = envelope;
    std::size_t winning_id = 0;bool changed = false;
    for (std::size_t m = 1; m < minima.size() && attempted < c.coordination_max_candidates; ++m) {
      const std::size_t a = minima[m - 1], b = minima[m];
      if (envelope.distance[b] - envelope.distance[a] > c.coordination_max_gap) {continue;}
      const auto group = std::find_if(
        groups.begin(), groups.end(), [&](const auto & span) {
          return envelope.distance[a] >= span.first - 1e-8 &&
          envelope.distance[b] <= span.second + 1e-8;
        });
      if (group == groups.end()) {continue;}
      const std::size_t begin = a / 2, end = (b + 1) / 2;
      if (end <= begin || std::any_of(
          path.begin() + begin + 1, path.begin() + end + 1,
          [](const auto & p) {return p.stop_required;})) {continue;}
      const double peak = *std::max_element(
        envelope.speed.begin() + a,
        envelope.speed.begin() + b + 1);
      std::vector<double> floors{std::max(envelope.speed[a], envelope.speed[b])};
      if (std::abs(envelope.speed[a] - envelope.speed[b]) > 1e-10) {
        floors.push_back(std::min(envelope.speed[a], envelope.speed[b]));
      }
      for (double floor:floors) {
        if (peak - floor < c.coordination_min_gain) {continue;}
        for (double fraction:c.coordination_trim_fractions) {
          if (attempted >= c.coordination_max_candidates) {break;}
          const auto id = attempted++;
          const double cap = peak - fraction * (peak - floor);
          auto candidate = path;
          for (std::size_t i = begin; i < end; ++i) {
            candidate[i].speed_limit = std::min(candidate[i].speed_limit, cap);
          }
          const auto e = speed_envelope(path_speed_segments(candidate), c.motion);
          const auto events = speed_envelope_events(e, c);
          const double variation = envelope_variation(e);
          const Span span{envelope.distance[a], envelope.distance[b]};
          bool local = true;
          for (const auto & prior:groups) {
            local = local && local_time_ok(original, e, prior, c.coordination_max_time_loss);
          }
          const bool time = e.time <= original.time * (1 + c.coordination_max_time_loss) + 1e-8 &&
            e.time + alignment_time <= time_limit + 1e-8;
          const bool gain = events < before_events || (events == before_events &&
            variation <= before_variation - c.coordination_min_gain);
          if (id) {trials << ',';}
          trials << "{\"id\":" << id << ",\"begin_s\":" << span.first << ",\"end_s\":" <<
            span.second << ",\"target_floor\":" << floor << ",\"fraction\":" << fraction <<
            ",\"time\":" << e.time <<
            ",\"events\":" << events << ",\"group\":" << (group - groups.begin()) <<
            ",\"variation\":" << variation << ",\"result\":\"" <<
            (!time ? "global_time" : !local ? "local_time" : !gain ? "no_gain" : "eligible") <<
            "\"}";
          const auto score = std::make_tuple(events, variation, e.time);
          if (time && local && gain && score < winning_score) {
            winning_score = score;winning_path = std::move(candidate);winning_envelope = e;
            winning_id = id;changed = true;
          }
        }
      }
    }
    if (!changed) {break;}
    path = std::move(winning_path);envelope = std::move(winning_envelope);
    selected.push_back(winning_id);
    if (attempted >= c.coordination_max_candidates) {break;}
  }
  trials << ']';std::ostringstream out;out.precision(12);
  out << "{\"policy\":\"bounded\",\"base_time\":" << original.time <<
    ",\"final_time\":" << envelope.time << ",\"time_loss\":" <<
    (original.time > 0 ? envelope.time / original.time - 1 : 0) << ",\"selected\":[";
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (i) {
      out << ',';
    }
    out << selected[i];
  }
  out << "],\"groups\":[";
  for (std::size_t i = 0; i < groups.size(); ++i) {
    if (i) {out << ',';}
    const auto span = groups[i];
    out << "{\"begin_s\":" << span.first << ",\"end_s\":" << span.second <<
      ",\"base_time\":" << envelope_interval_time(original, span.first, span.second) <<
      ",\"final_time\":" << envelope_interval_time(envelope, span.first, span.second) << '}';
  }
  out << "],\"trials\":" << trials.str() << '}';report = out.str();
}
}  // namespace plan

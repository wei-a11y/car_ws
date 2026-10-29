// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/lqr_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace controller
{
void LqrTracker::set_profile(const std::vector<ReferencePoint> & path, bool continuous_curvature)
{
  clear_path();
  if (path.empty()) {
    return;
  }
  if (path.size() > config_.max_path_points || !path.back().stop_required) {
    throw std::invalid_argument("Profile exceeds point limit or has no terminal stop");
  }
  std::vector<Segment> segments;
  std::vector<double> lengths(path.size(), 0.0), limits(path.size(), config_.v_max);
  std::vector<double> acceleration(path.size(), config_.linear_acceleration);
  std::vector<double> deceleration(path.size(), config_.linear_deceleration);
  std::vector<double> segment_limits(path.size(), config_.v_max);
  std::vector<Point2D> points;
  const auto motion = motion_limits(config_);
  std::vector<plan::SpeedSegment> budget_segments;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & point = path[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.yaw) ||
      !std::isfinite(point.curvature) ||
      std::abs(point.curvature) > config_.max_reference_curvature)
    {
      throw std::invalid_argument("Invalid reference geometry or curvature");
    }
    if (!std::isfinite(point.speed_limit) || point.speed_limit < -1.0 ||
      (point.speed_limit < 0.0 && point.speed_limit != -1.0) ||
      !std::isfinite(point.curvature_bound) || point.curvature_bound < 0.0 ||
      !std::isfinite(point.curvature_rate_bound) || point.curvature_rate_bound < 0.0 ||
      point.segment_type > 2 ||
      (point.speed_limit >= 0.0 && point.curvature_bound + 1e-8 < std::abs(point.curvature)))
    {
      throw std::invalid_argument("Invalid extended reference limits or segment type");
    }
    if (point.speed_limit >= 0.0) {limits[i] = std::min(limits[i], point.speed_limit);}
    points.push_back({point.x, point.y});
    // Include incoming and outgoing curvature so braking begins before an entry transition.
    const double curvature = std::max(
      std::abs(point.curvature),
      i > 0 && !path[i - 1].stop_required ? std::abs(path[i - 1].curvature) : 0.0);
    if (curvature > 0.0) {
      limits[i] = std::min(
        {limits[i],
          config_.feedforward_angular_fraction * config_.w_max / curvature,
          std::sqrt(config_.lateral_acceleration / curvature)});
    }
    if (point.stop_required) {
      limits[i] = 0.0;
    }
    if (i == 0) {
      continue;
    }
    const double dx = point.x - path[i - 1].x, dy = point.y - path[i - 1].y;
    const double length = std::hypot(dx, dy), yaw = std::atan2(dy, dx);
    lengths[i] = lengths[i - 1] + length;
    if (!std::isfinite(lengths[i]) || length <= 0.0 ||
      std::abs(normalize_angle(path[i - 1].yaw - yaw)) > config_.reference_heading_tolerance ||
      (!point.stop_required &&
      std::abs(normalize_angle(point.yaw - yaw)) > config_.reference_heading_tolerance))
    {
      throw std::invalid_argument("Reference tangents disagree with Path or duplicate points");
    }
    segments.push_back({points[i - 1], points[i], length, yaw, lengths[i - 1]});
    const auto & previous = path[i - 1];
    const double peak = std::max(
      {std::abs(previous.curvature), std::abs(point.curvature),
        previous.curvature_bound});
    if (config_.enable_dynamic_angular_budget && !continuous_curvature && !point.stop_required &&
      std::abs(point.curvature - previous.curvature) > 1e-10)
    {
      throw std::invalid_argument("Dynamic budget requires continuous nodal curvature");
    }
    const double measured_rate = continuous_curvature && !point.stop_required ?
      std::abs(point.curvature - previous.curvature) / length : 0.0;
    const auto bound = plan::curve_limits(
      motion, peak,
      std::max(measured_rate, previous.curvature_rate_bound));
    acceleration[i] = bound.acceleration;
    deceleration[i] = bound.deceleration;
    const double cap = previous.speed_limit >= 0.0 ?
      std::min(bound.speed, previous.speed_limit) : bound.speed;
    limits[i - 1] = std::min(limits[i - 1], cap);
    limits[i] = std::min(limits[i], cap);
    segment_limits[i] = cap;
    budget_segments.push_back(
      {length, peak,
        std::max(measured_rate, previous.curvature_rate_bound), cap, point.stop_required});
  }
  // Backward braking envelope incorporates every future curve and mandatory stop.
  for (std::size_t i = path.size() - 1; i > 0; --i) {
    limits[i - 1] = std::min(
      limits[i - 1], std::sqrt(
        limits[i] * limits[i] + 2.0 * deceleration[i] * (lengths[i] - lengths[i - 1])));
  }
  budget_geometry_ = budget_segments;
  if (config_.enable_dynamic_angular_budget && !budget_segments.empty()) {
    const auto original = budget_segments;
    for (std::size_t i = 0; i < budget_segments.size(); ++i) {
      // The complete outgoing segment must already support its preview geometry.
      // This moves braking upstream before the preview enters a faster-changing curve.
      for (std::size_t j = i + 1; j < original.size() && !original[j - 1].stop_after &&
        lengths[j] <= lengths[i + 1] + config_.feedforward_preview_distance; ++j)
      {
        budget_segments[i].curvature =
          std::max(budget_segments[i].curvature, original[j].curvature);
        budget_segments[i].rate = std::max(budget_segments[i].rate, original[j].rate);
        budget_segments[i].cap = std::min(budget_segments[i].cap, original[j].cap);
      }
    }
    budget_envelope_ = plan::speed_envelope(budget_segments, motion, false);
    budget_fallback_ = budget_envelope_.conservative_fallback;
    for (std::size_t i = 0; i < limits.size(); ++i) {
      limits[i] = budget_envelope_.speed[2 * i];
    }
  }
  budget_segments_ = std::move(budget_segments);
  points_ = std::move(points);
  segments_ = std::move(segments);
  arc_lengths_ = std::move(lengths);
  speed_limits_ = std::move(limits);
  reference_ = path;
  acceleration_limits_ = std::move(acceleration);
  deceleration_limits_ = std::move(deceleration);
  segment_speed_limits_ = std::move(segment_limits);
  continuous_curvature_ = continuous_curvature;
  next_stop_ = 1;
  while (next_stop_ < path.size() && !path[next_stop_].stop_required) {
    ++next_stop_;
  }
  if (path.size() == 1) {
    next_stop_ = 0;
  }
  continuous_ = true;
}

LqrTracker::Sample LqrTracker::reference_at(double s) const
{
  const auto upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
  auto index = upper == arc_lengths_.begin() ? 0U :
    static_cast<std::size_t>(upper - arc_lengths_.begin() - 1);
  index = std::clamp(index, phase_begin_, next_stop_ - 1);
  const auto & segment = segments_[index];
  const double ratio = std::clamp((s - segment.start_s) / segment.length, 0.0, 1.0);
  const double end_yaw =
    reference_[index + 1].stop_required ? segment.yaw : reference_[index + 1].yaw;
  const double yaw = reference_[index].yaw +
    ratio * normalize_angle(end_yaw - reference_[index].yaw);
  return {{segment.start.x + ratio * (segment.end.x - segment.start.x),
    segment.start.y + ratio * (segment.end.y - segment.start.y)},
    yaw, continuous_curvature_ && !reference_[index + 1].stop_required ?
    reference_[index].curvature + ratio *
    (reference_[index + 1].curvature - reference_[index].curvature) :
    reference_[index].curvature, index};
}

TrackerResult LqrTracker::step_continuous(Pose2D pose, Velocity2D actual)
{
  TrackerResult result;
  auto & d = result.diagnostics;
  d.pose = pose;
  d.actual = actual;
  d.segment_count = segments_.size();
  d.aligning = aligning_;
  d.next_stop_index = next_stop_;
  const auto fault = [&](TrackerFault reason) {
      result.command = {0.0, 0.0};
      fault_ = true;
      d.fault = reason;
      fault_diagnostics_ = d;
      result.state = TrackerState::TRACKING_ERROR;
      return result;
    };
  if (fault_) {
    result.state = TrackerState::TRACKING_ERROR;
    d = fault_diagnostics_;
    return result;
  }
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
    !std::isfinite(actual.v) || !std::isfinite(actual.w))
  {
    return fault(TrackerFault::NONFINITE_INPUT);
  }
  if (reached_) {
    result.state = TrackerState::GOAL_REACHED;
    return result;
  }
  const bool stopped = std::abs(actual.v) <= config_.stopped_linear_tolerance &&
    std::abs(actual.w) <= config_.stopped_angular_tolerance;
  if (segments_.empty()) {
    d.endpoint_distance = std::hypot(pose.x - points_.front().x, pose.y - points_.front().y);
    if (d.endpoint_distance > config_.goal_position_tolerance) {
      return fault(TrackerFault::SINGLE_POINT_NOT_REACHED);
    }
    reached_ = stopped;
    result.state = stopped ? TrackerState::GOAL_REACHED : TrackerState::BRAKING;
    return result;
  }
  const double stop_s = arc_lengths_[next_stop_];
  const double low = std::max(
    arc_lengths_[phase_begin_],
    continuous_progress_ - config_.nearest_backward_distance);
  const double high = std::min(stop_s, continuous_progress_ + config_.nearest_forward_distance);
  double best_s = low, best_distance = std::numeric_limits<double>::infinity();
  auto first = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), low);
  std::size_t begin = first == arc_lengths_.begin() ? 0U : first - arc_lengths_.begin() - 1;
  begin = std::max(begin, phase_begin_);
  for (std::size_t i = begin; i < next_stop_ && segments_[i].start_s <= high; ++i) {
    const auto & segment = segments_[i];
    const double c = std::cos(segment.yaw), sn = std::sin(segment.yaw);
    const double projected = (pose.x - segment.start.x) * c + (pose.y - segment.start.y) * sn;
    const double local = std::clamp(
      projected,
      std::max(0.0, low - segment.start_s), std::min(segment.length, high - segment.start_s));
    const double distance = std::hypot(
      pose.x - segment.start.x - local * c,
      pose.y - segment.start.y - local * sn);
    if (distance < best_distance) {
      best_distance = distance;
      best_s = segment.start_s + local;
    }
  }
  const auto reference = reference_at(best_s);
  const auto & segment = segments_[reference.segment];
  continuous_progress_ = std::max(continuous_progress_, best_s);
  result.progress = continuous_progress_;
  result.remaining = stop_s - continuous_progress_;
  result.nearest = reference.point;
  result.lookahead = reference_at(std::min(stop_s, best_s + config_.lookahead_distance)).point;
  result.lateral_error = -std::sin(reference.yaw) * (pose.x - reference.point.x) +
    std::cos(reference.yaw) * (pose.y - reference.point.y);
  result.heading_error = normalize_angle(pose.yaw - reference.yaw);
  d.segment_index = reference.segment;
  d.start = segment.start;
  d.end = segment.end;
  d.nearest = reference.point;
  d.length = segment.length;
  d.projected = (pose.x - segment.start.x) * std::cos(segment.yaw) +
    (pose.y - segment.start.y) * std::sin(segment.yaw);
  d.nearest_distance = best_distance;
  d.endpoint_distance = std::hypot(pose.x - points_[next_stop_].x, pose.y - points_[next_stop_].y);
  d.lateral_error = result.lateral_error;
  d.heading_error = result.heading_error;
  d.geometry_valid = true;
  if (best_distance > config_.max_tracking_error) {
    return fault(TrackerFault::NEAREST_DISTANCE_EXCEEDED);
  }
  if (d.endpoint_distance <= config_.goal_position_tolerance) {
    result.state = TrackerState::BRAKING;
    if (stopped) {
      if (next_stop_ + 1 == points_.size()) {
        reached_ = true;
        result.state = TrackerState::GOAL_REACHED;
      } else {
        phase_begin_ = next_stop_;
        continuous_progress_ = stop_s;
        do {
          ++next_stop_;
        } while (!reference_[next_stop_].stop_required);
        aligning_ = true;
        result.state = TrackerState::ALIGNING;
      }
    }
    return result;
  }
  const auto & terminal = segments_[next_stop_ - 1];
  if (best_s >= stop_s && (pose.x - terminal.end.x) * std::cos(terminal.yaw) +
    (pose.y - terminal.end.y) * std::sin(terminal.yaw) > 0.0)
  {
    return fault(TrackerFault::SEGMENT_END_OVERSHOOT);
  }
  if (std::abs(result.heading_error) > config_.tracking_align_trigger) {
    aligning_ = true;
  }
  if (aligning_) {
    result.state = TrackerState::ALIGNING;
    if (std::abs(actual.v) > config_.stopped_linear_tolerance) {
      result.state = TrackerState::BRAKING;
      return result;
    }
    if (std::abs(result.heading_error) > config_.heading_tolerance) {
      result.command.w = std::clamp(
        -config_.align_gain * result.heading_error,
        -config_.w_max, config_.w_max);
      return result;
    }
    if (!stopped) {
      return result;
    }
    aligning_ = false;
  }
  const double preview_s = std::min(stop_s, best_s + config_.feedforward_preview_distance);
  const auto preview_reference = reference_at(preview_s);
  const double end_distance = std::max(
    0.0,
    arc_lengths_[preview_reference.segment + 1] - preview_s -
    std::max(0.0, actual.v) * config_.control_period);
  const double deceleration = deceleration_limits_[preview_reference.segment + 1];
  double limit = std::min(
    {config_.v_max, config_.slowdown_gain * result.remaining,
      std::sqrt(
        std::pow(speed_limits_[preview_reference.segment + 1], 2) +
        2.0 * deceleration * end_distance)});
  // A long preview must not skip a tight bend between the current and preview points.
  const double current_braking_distance = std::max(
    0.0, arc_lengths_[reference.segment + 1] - best_s -
    std::max(0.0, actual.v) * config_.control_period);
  limit = std::min(
    limit, std::sqrt(
      std::pow(speed_limits_[reference.segment + 1], 2) +
      2.0 * deceleration_limits_[reference.segment + 1] * current_braking_distance));
  // Respect both endpoint caps, including curvature-rate limits within a transition.
  limit = std::min(
    limit, std::sqrt(
      std::pow(speed_limits_[preview_reference.segment], 2) +
      2.0 * acceleration_limits_[preview_reference.segment + 1] *
      std::max(0.0, preview_s - arc_lengths_[preview_reference.segment])));
  const double curvature = preview_reference.curvature;
  const double limiting_curvature = std::max(std::abs(reference.curvature), std::abs(curvature));
  limit = std::min(
    {limit, segment_speed_limits_[reference.segment + 1],
      segment_speed_limits_[preview_reference.segment + 1]});
  if (limiting_curvature > 0.0) {
    limit = std::min(
      {limit, config_.feedforward_angular_fraction * config_.w_max / limiting_curvature,
        std::sqrt(config_.lateral_acceleration / limiting_curvature)});
  }
  result.state = TrackerState::TRACKING;
  double acceleration = std::min(
    acceleration_limits_[reference.segment + 1],
    acceleration_limits_[preview_reference.segment + 1]);
  double deceleration_limit = std::min(deceleration_limits_[reference.segment + 1], deceleration);
  double k_bound = 0.0, q_bound = 0.0;
  if (config_.enable_dynamic_angular_budget) {
    auto motion = motion_limits(config_);
    motion.enable_dynamic_angular_budget = !budget_fallback_;
    d.dynamic_budget_fallback = budget_fallback_;
    // Include every crossed segment: a preview must never skip an intervening bottleneck.
    limit = std::min(config_.v_nominal, config_.slowdown_gain * result.remaining);
    for (std::size_t i = reference.segment; i <= preview_reference.segment; ++i) {
      const auto & edge = budget_geometry_[i];
      k_bound = std::max(k_bound, edge.curvature); q_bound = std::max(q_bound, edge.rate);
      limit = std::min(limit, edge.cap);
    }
    // Brake to the next virtual node, including an interior peak between two stops.
    auto upper = std::upper_bound(
      budget_envelope_.distance.begin(),
      budget_envelope_.distance.end(), best_s);
    const auto node = std::min<std::size_t>(
      upper - budget_envelope_.distance.begin(), budget_envelope_.distance.size() - 1);
    const auto & edge = budget_segments_[reference.segment];
    limit = std::min(
      limit, plan::reachable_speed(
        motion, budget_envelope_.speed[node],
        std::max(
          0.0,
          budget_envelope_.distance[node] - best_s -
          std::max(0.0, actual.v) * config_.control_period),
        edge.curvature, edge.rate, edge.cap, true));
    const double budget = plan::angular_budget(motion) / (budget_fallback_ ? 2.0 : 1.0);
    const double remaining_budget = budget - std::pow(std::max(0.0, actual.v), 2) * q_bound;
    if (remaining_budget < -1e-8) {return fault(TrackerFault::ANGULAR_BUDGET_EXCEEDED);}
    if (k_bound > 0) {
      acceleration =
        std::min(config_.linear_acceleration, std::max(0.0, remaining_budget) / k_bound);
      deceleration_limit = std::min(
        config_.linear_deceleration, std::max(
          0.0,
          remaining_budget) /
        k_bound);
    } else {
      acceleration = config_.linear_acceleration;deceleration_limit = config_.linear_deceleration;
    }
  }
  const double speed = std::min(config_.v_nominal, limit);
  result.reference_speed = speed;
  const double prior_command = speed_initialized_ ? previous_command_ : std::max(0.0, actual.v);
  result.command.v = speed_control(speed, actual.v, limit, acceleration, deceleration_limit);
  if (config_.enable_dynamic_angular_budget) {
    const auto demand = [&](double command) {
        // Feedforward acceleration is the command derivative. The measured speed
        // bounds progress through curvature; velocity tracking error is not acceleration.
        const double a = std::abs(command - prior_command) / config_.control_period;
        return a * k_bound +
               std::pow(std::max({0.0, command, actual.v, prior_command}), 2) * q_bound;
      };
    const double budget = plan::angular_budget(motion_limits(config_));
    if (demand(result.command.v) > budget + 1e-8) {
      // The feasible set is convex in command speed. Search from the measured/previous centre.
      const double centre = std::clamp(prior_command, 0.0, limit);
      if (demand(centre) > budget + 1e-8) {return fault(TrackerFault::ANGULAR_BUDGET_EXCEEDED);}
      double low = 0, high = 1;
      const double proposed = result.command.v;
      for (std::size_t iteration = 0; iteration < config_.speed_search_iterations; ++iteration) {
        const double fraction = (low + high) / 2;
        if (demand(centre + fraction * (proposed - centre)) <= budget) {low = fraction;} else {
          high = fraction;
        }
      }
      result.command.v = centre + low * (proposed - centre);
      previous_command_ = result.command.v;
      previous_reference_ = std::min(previous_reference_, result.command.v);
    }
    const double a = std::abs(result.command.v - prior_command) / config_.control_period;
    d.linear_angular_demand = a * k_bound;
    d.curvature_angular_demand = std::pow(
      std::max(
        {0.0, result.command.v, actual.v,
          prior_command}), 2) * q_bound;
  }
  result.reference_curvature = d.reference_curvature = curvature;
  result.feedforward_angular = d.feedforward_angular = result.command.v * curvature;
  // Precomputed speed schedule avoids a Riccati solve in every control callback.
  const double range = config_.v_max - config_.gain_schedule_min_speed;
  const double position = range > 0.0 ? std::clamp(
    (result.command.v - config_.gain_schedule_min_speed) / range, 0.0, 1.0) *
    (scheduled_gains_.size() - 1) : 0.0;
  const auto bin = std::min(static_cast<std::size_t>(position), scheduled_gains_.size() - 2);
  const double ratio = position - bin;
  const std::array<double, 2> scheduled_gain{
    (1.0 - ratio) * scheduled_gains_[bin][0] + ratio * scheduled_gains_[bin + 1][0],
    (1.0 - ratio) * scheduled_gains_[bin][1] + ratio * scheduled_gains_[bin + 1][1]};
  const double angular = std::clamp(
    result.feedforward_angular - scheduled_gain[0] * result.lateral_error -
    scheduled_gain[1] * result.heading_error, -config_.w_max, config_.w_max);
  const double previous_angular = std::clamp(actual.w, -config_.w_max, config_.w_max);
  result.command.w = std::clamp(
    angular,
    std::max(
      -config_.w_max,
      previous_angular - config_.angular_acceleration * config_.control_period),
    std::min(
      config_.w_max,
      previous_angular + config_.angular_acceleration * config_.control_period));
  return result;
}
}  // namespace controller

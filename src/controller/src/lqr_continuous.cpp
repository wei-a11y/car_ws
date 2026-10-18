// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/lqr_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace controller
{
void LqrTracker::set_profile(const std::vector<ReferencePoint> & path)
{
  clear_path();
  if (path.empty()) {
    return;
  }
  if (path.size() > config_.max_path_points || !path.back().stop_required) {
    throw std::invalid_argument("Profile exceeds point limit or has no terminal stop");
  }
  std::vector<Segment> segments;
  std::vector<double> lengths(path.size(), 0.0), limits(path.size(), config_.v_nominal);
  std::vector<Point2D> points;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & point = path[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.yaw) ||
      !std::isfinite(point.curvature) ||
      std::abs(point.curvature) > config_.max_reference_curvature)
    {
      throw std::invalid_argument("Invalid reference geometry or curvature");
    }
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
  }
  // Backward braking envelope incorporates every future curve and mandatory stop.
  for (std::size_t i = path.size() - 1; i > 0; --i) {
    limits[i - 1] = std::min(
      limits[i - 1], std::sqrt(
        limits[i] * limits[i] + 2.0 * config_.linear_deceleration * (lengths[i] - lengths[i - 1])));
  }
  points_ = std::move(points);
  segments_ = std::move(segments);
  arc_lengths_ = std::move(lengths);
  speed_limits_ = std::move(limits);
  reference_ = path;
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
    yaw, reference_[index].curvature, index};
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
  double speed = std::min(
    {config_.v_nominal, config_.slowdown_gain * result.remaining,
      std::sqrt(
        std::pow(speed_limits_[preview_reference.segment + 1], 2) +
        2.0 * config_.linear_deceleration * end_distance)});
  const double curvature = preview_reference.curvature;
  const double limiting_curvature = std::max(std::abs(reference.curvature), std::abs(curvature));
  if (limiting_curvature > 0.0) {
    speed = std::min(
      {speed, config_.feedforward_angular_fraction * config_.w_max / limiting_curvature,
        std::sqrt(config_.lateral_acceleration / limiting_curvature)});
  }
  result.state = TrackerState::TRACKING;
  result.reference_speed = speed;
  result.command.v = std::clamp(
    std::min(
      speed,
      std::max(
        0.0,
        actual.v) + config_.linear_acceleration * config_.control_period), 0.0,
    config_.v_max);
  result.reference_curvature = d.reference_curvature = curvature;
  result.feedforward_angular = d.feedforward_angular = result.command.v * curvature;
  const double angular = std::clamp(
    result.feedforward_angular - gain_[0] * result.lateral_error -
    gain_[1] * result.heading_error, -config_.w_max, config_.w_max);
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

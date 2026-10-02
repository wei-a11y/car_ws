// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/lqr_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace controller
{
namespace
{
const double kPi = std::acos(-1.0);

bool finite(Point2D point) {return std::isfinite(point.x) && std::isfinite(point.y);}
double distance(Point2D a, Point2D b) {return std::hypot(a.x - b.x, a.y - b.y);}
bool same(Point2D a, Point2D b) {return a.x == b.x && a.y == b.y;}

bool forward_collinear(Point2D a, Point2D b, Point2D c)
{
  const double ux = b.x - a.x, uy = b.y - a.y, vx = c.x - b.x, vy = c.y - b.y;
  const double product = std::hypot(ux, uy) * std::hypot(vx, vy);
  constexpr double kRoundoff = 64.0 * std::numeric_limits<double>::epsilon();
  const double coordinate_scale = std::max(
    {1.0, std::abs(a.x), std::abs(a.y),
      std::abs(b.x), std::abs(b.y), std::abs(c.x), std::abs(c.y)});
  const double tolerance = kRoundoff * coordinate_scale * (std::hypot(ux, uy) + std::hypot(vx, vy));
  return std::isfinite(product) && std::abs(ux * vy - uy * vx) <= tolerance &&
         ux * vx + uy * vy > 0.0;
}
}  // namespace

double normalize_angle(double angle)
{
  const double wrapped = std::remainder(angle, 2.0 * kPi);
  return wrapped >= kPi ? wrapped - 2.0 * kPi : wrapped;
}

void validate_config(const TrackerConfig & c)
{
  const double positive[] = {c.control_period, c.v_nominal, c.v_max, c.w_max,
    c.q_lateral, c.q_heading, c.r_angular, c.riccati_tolerance, c.lookahead_distance,
    c.nearest_forward_distance, c.nearest_backward_distance, c.slowdown_gain, c.align_gain,
    c.heading_tolerance, c.align_trigger, c.goal_position_tolerance,
    c.stopped_linear_tolerance, c.stopped_angular_tolerance, c.max_tracking_error};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Tracker numeric parameters must be finite and positive");
    }
  }
  if (c.v_nominal > c.v_max || c.heading_tolerance >= c.align_trigger ||
    c.align_trigger >= kPi / 2.0 || c.riccati_tolerance >= 1.0 ||
    c.riccati_max_iterations == 0 || c.max_path_points == 0)
  {
    throw std::invalid_argument("Inconsistent tracker limits, alignment thresholds or solver limits");
  }
}

DiscreteModel discrete_model(double speed, double period)
{
  if (!std::isfinite(speed) || speed < 0.0 || !std::isfinite(period) || period <= 0.0) {
    throw std::invalid_argument("Model speed must be nonnegative and period positive");
  }
  const DiscreteModel model{{1.0, speed * period, 0.0, 1.0},
    {0.5 * speed * period * period, period}};
  if (!std::isfinite(model.a[1]) || !std::isfinite(model.b[0])) {
    throw std::invalid_argument("Discrete model overflow");
  }
  return model;
}

std::array<double, 2> lqr_gain(const TrackerConfig & config)
{
  validate_config(config);
  const auto model = discrete_model(config.v_nominal, config.control_period);
  const double a = model.a[1], b0 = model.b[0], b1 = model.b[1];
  double p00 = config.q_lateral, p01 = 0.0, p11 = config.q_heading;
  bool converged = false;
  for (std::size_t i = 0; i < config.riccati_max_iterations; ++i) {
    const double pb0 = p00 * b0 + p01 * b1, pb1 = p01 * b0 + p11 * b1;
    const double denominator = config.r_angular + b0 * pb0 + b1 * pb1;
    const double right = a * pb0 + pb1;
    const double n00 = config.q_lateral + p00 - pb0 * pb0 / denominator;
    const double n01 = a * p00 + p01 - pb0 * right / denominator;
    const double n11 = config.q_heading + a * a * p00 + 2.0 * a * p01 + p11 -
      right * right / denominator;
    if (!std::isfinite(denominator) || denominator <= 0.0 ||
      !std::isfinite(n00) || !std::isfinite(n01) || !std::isfinite(n11))
    {
      throw std::runtime_error("Riccati iteration is not finite");
    }
    const double residual = std::max(
      {std::abs(n00 - p00), std::abs(n01 - p01),
        std::abs(n11 - p11)}) / std::max({1.0, std::abs(n00), std::abs(n01), std::abs(n11)});
    p00 = n00; p01 = n01; p11 = n11;
    if (residual <= config.riccati_tolerance) {
      converged = true;
      break;
    }
  }
  if (!converged) {
    throw std::runtime_error("Riccati iteration did not converge");
  }
  const double pb0 = p00 * b0 + p01 * b1, pb1 = p01 * b0 + p11 * b1;
  const double denominator = config.r_angular + b0 * pb0 + b1 * pb1;
  const std::array<double, 2> gain{pb0 / denominator, (a * pb0 + pb1) / denominator};
  // Jury conditions for this ZOH straight-segment model: stable at every fixed
  // 0 < v <= v_nominal. At v=0 the lateral pole is 1, so no zero-speed DARE is used.
  const double t = config.control_period;
  if (!std::isfinite(gain[0]) || !std::isfinite(gain[1]) || gain[0] <= 0.0 ||
    t * gain[1] >= 2.0 || gain[1] <= 0.5 * config.v_nominal * t * gain[0])
  {
    throw std::runtime_error("LQR gain fails local discrete stability checks");
  }
  return gain;
}

const char * state_name(TrackerState state)
{
  switch (state) {
    case TrackerState::WAIT_PATH: return "WAIT_PATH";
    case TrackerState::ALIGNING: return "ALIGNING";
    case TrackerState::TRACKING: return "TRACKING";
    case TrackerState::BRAKING: return "BRAKING";
    case TrackerState::GOAL_REACHED: return "GOAL_REACHED";
    case TrackerState::TRACKING_ERROR: return "TRACKING_ERROR";
  }
  return "TRACKING_ERROR";
}

LqrTracker::LqrTracker(TrackerConfig config)
: config_(config), gain_(lqr_gain(config)) {}

void LqrTracker::clear_path()
{
  points_.clear(); segments_.clear(); segment_ = 0; progress_ = 0.0;
  aligning_ = true; fault_ = false; reached_ = false;
}

void LqrTracker::set_path(const std::vector<Point2D> & path)
{
  clear_path();
  if (path.size() > config_.max_path_points) {
    throw std::invalid_argument("Path exceeds max_path_points");
  }
  std::vector<Point2D> points;
  for (const auto & point : path) {
    if (!finite(point)) {
      throw std::invalid_argument("Path coordinates are not finite");
    }
    if (!points.empty() && same(points.back(), point)) {
      continue;
    }
    while (points.size() >= 2 &&
      forward_collinear(points[points.size() - 2], points.back(), point))
    {
      points.pop_back();
    }
    points.push_back(point);
  }
  std::vector<Segment> segments;
  double start_s = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const auto a = points[i - 1], b = points[i];
    const double length = distance(a, b);
    if (!std::isfinite(length) || length <= 0.0 || !std::isfinite(start_s + length)) {
      throw std::invalid_argument("Path segment length is invalid");
    }
    segments.push_back({a, b, length, std::atan2(b.y - a.y, b.x - a.x), start_s});
    start_s += length;
  }
  points_ = std::move(points);
  segments_ = std::move(segments);
}

TrackerResult LqrTracker::step(Pose2D pose, Velocity2D actual)
{
  TrackerResult result;
  if (!has_path()) {
    return result;
  }
  if (!finite({pose.x, pose.y}) || !std::isfinite(pose.yaw) ||
    !std::isfinite(actual.v) || !std::isfinite(actual.w))
  {
    fault_ = true;
  }
  if (fault_) {
    result.state = TrackerState::TRACKING_ERROR;
    return result;
  }
  if (reached_) {
    result.state = TrackerState::GOAL_REACHED;
    return result;
  }
  const bool stopped = std::abs(actual.v) <= config_.stopped_linear_tolerance &&
    std::abs(actual.w) <= config_.stopped_angular_tolerance;
  const Point2D position{pose.x, pose.y};
  if (segments_.empty()) {
    result.nearest = result.lookahead = points_.front();
    if (distance(position, points_.front()) > config_.goal_position_tolerance) {
      fault_ = true;
      result.state = TrackerState::TRACKING_ERROR;
    } else {
      reached_ = stopped;
      result.state = stopped ? TrackerState::GOAL_REACHED : TrackerState::BRAKING;
    }
    return result;
  }
  const auto & segment = segments_[segment_];
  const double c = std::cos(segment.yaw), s = std::sin(segment.yaw);
  const double projected = (pose.x - segment.start.x) * c + (pose.y - segment.start.y) * s;
  const double nearest_s = std::clamp(
    projected,
    std::max(0.0, progress_ - config_.nearest_backward_distance),
    std::min(segment.length, progress_ + config_.nearest_forward_distance));
  result.nearest = {segment.start.x + c * nearest_s, segment.start.y + s * nearest_s};
  result.lateral_error = -s * (pose.x - result.nearest.x) + c * (pose.y - result.nearest.y);
  result.heading_error = normalize_angle(pose.yaw - segment.yaw);
  progress_ = std::max(progress_, nearest_s);
  result.progress = segment.start_s + progress_;
  result.remaining = segment.length - progress_;
  const double preview = std::min(segment.length, nearest_s + config_.lookahead_distance);
  result.lookahead = {segment.start.x + c * preview, segment.start.y + s * preview};
  if (distance(position, result.nearest) > config_.max_tracking_error) {
    fault_ = true;
    result.state = TrackerState::TRACKING_ERROR;
    return result;
  }
  if (distance(position, segment.end) <= config_.goal_position_tolerance) {
    result.state = TrackerState::BRAKING;
    if (stopped) {
      if (segment_ + 1 == segments_.size()) {
        reached_ = true;
        result.state = TrackerState::GOAL_REACHED;
      } else {
        ++segment_; progress_ = 0.0; aligning_ = true;
        result.state = TrackerState::ALIGNING;
      }
    }
    return result;
  }
  if (projected >= segment.length) {
    fault_ = true;  // An end projection is not proof of reaching the actual endpoint.
    result.state = TrackerState::TRACKING_ERROR;
    return result;
  }
  if (std::abs(result.heading_error) > config_.align_trigger) {
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
  result.state = TrackerState::TRACKING;
  result.reference_speed = std::min(config_.v_nominal, config_.slowdown_gain * result.remaining);
  result.command.v = std::clamp(result.reference_speed, 0.0, config_.v_max);
  result.command.w = std::clamp(
    -gain_[0] * result.lateral_error - gain_[1] * result.heading_error,
    -config_.w_max, config_.w_max);
  return result;
}
}  // namespace controller

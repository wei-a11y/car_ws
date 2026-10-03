// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef CONTROLLER__LQR_TRACKER_HPP_
#define CONTROLLER__LQR_TRACKER_HPP_

#include <array>
#include <cstddef>
#include <vector>

namespace controller
{
struct Point2D {double x; double y;};
struct Pose2D {double x; double y; double yaw;};
struct Velocity2D {double v; double w;};

struct TrackerConfig
{
  double control_period;
  double v_nominal;
  double v_max;
  double w_max;
  double q_lateral;
  double q_heading;
  double r_angular;
  double riccati_tolerance;
  std::size_t riccati_max_iterations;
  double lookahead_distance;
  double nearest_forward_distance;
  double nearest_backward_distance;
  double slowdown_gain;
  double align_gain;
  double heading_tolerance;
  double align_trigger;
  double goal_position_tolerance;
  double stopped_linear_tolerance;
  double stopped_angular_tolerance;
  double max_tracking_error;
  std::size_t max_path_points;
};

struct DiscreteModel
{
  std::array<double, 4> a;  // row-major 2x2
  std::array<double, 2> b;
};

double normalize_angle(double angle);
void validate_config(const TrackerConfig & config);
DiscreteModel discrete_model(double speed, double period);
std::array<double, 2> lqr_gain(const TrackerConfig & config);

enum class TrackerState {WAIT_PATH, ALIGNING, TRACKING, BRAKING, GOAL_REACHED, TRACKING_ERROR};
const char * state_name(TrackerState state);

enum class TrackerFault
{
  NONE, NONFINITE_INPUT, SINGLE_POINT_NOT_REACHED,
  NEAREST_DISTANCE_EXCEEDED, SEGMENT_END_OVERSHOOT
};
const char * fault_name(TrackerFault fault);

// Diagnostic snapshot only; never used to relax or change control decisions.
struct TrackerDiagnostics
{
  TrackerFault fault = TrackerFault::NONE;
  std::size_t segment_index = 0;  // zero-based, segment evaluated by this step
  std::size_t segment_count = 0;
  bool aligning = false;
  bool geometry_valid = false;
  Pose2D pose{0.0, 0.0, 0.0};
  Velocity2D actual{0.0, 0.0};
  Point2D start{0.0, 0.0}, end{0.0, 0.0}, nearest{0.0, 0.0};
  double length = 0.0, projected = 0.0;
  double nearest_distance = 0.0, endpoint_distance = 0.0;
  double lateral_error = 0.0, heading_error = 0.0;
};

struct TrackerResult
{
  Velocity2D command{0.0, 0.0};
  TrackerState state = TrackerState::WAIT_PATH;
  double lateral_error = 0.0;
  double heading_error = 0.0;
  double progress = 0.0;
  double remaining = 0.0;  // to the next mandatory stop, not necessarily the final endpoint
  double reference_speed = 0.0;
  Point2D nearest{0.0, 0.0};
  Point2D lookahead{0.0, 0.0};
  TrackerDiagnostics diagnostics;
};

// ROS-independent, forward-only tracker. Every real bend is a mandatory stop/align point.
// TRACKING_ERROR and GOAL_REACHED latch zero until a new path. External data freshness is the
// wrapper's responsibility; this core uses actual velocity only for stop/alignment confirmation.
class LqrTracker
{
public:
  explicit LqrTracker(TrackerConfig config);
  void set_path(const std::vector<Point2D> & path);  // invalid input clears the old path, then throws
  void clear_path();
  bool has_path() const {return !points_.empty();}
  const std::array<double, 2> & gain() const {return gain_;}
  TrackerResult step(Pose2D pose, Velocity2D actual);

private:
  struct Segment {Point2D start; Point2D end; double length; double yaw; double start_s;};
  TrackerConfig config_;
  std::array<double, 2> gain_;
  std::vector<Point2D> points_;
  std::vector<Segment> segments_;
  std::size_t segment_ = 0;
  double progress_ = 0.0;
  bool aligning_ = true;
  bool fault_ = false;
  bool reached_ = false;
  TrackerDiagnostics fault_diagnostics_;  // retain the FIRST fault until clear_path
};
}  // namespace controller
#endif  // CONTROLLER__LQR_TRACKER_HPP_

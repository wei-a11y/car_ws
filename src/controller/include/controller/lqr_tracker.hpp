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
struct ReferencePoint
{
  double x, y, yaw, curvature;
  bool stop_required;
};

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
  double linear_acceleration = 0.10;
  double linear_deceleration = 0.15;
  double angular_acceleration = 1.0;
  double lateral_acceleration = 0.04;
  double feedforward_angular_fraction = 0.75;
  double feedforward_preview_distance = 0.02;
  double tracking_align_trigger = 1.0;
  double max_reference_curvature = 20.0;
  double reference_heading_tolerance = 0.20;
  double speed_pid_kp = 0.4;
  double speed_pid_ki = 0.2;
  double speed_pid_kd = 0.0;
  double speed_pid_integral_limit = 0.10;  // limit the integral contribution, m/s
  double speed_pid_correction_limit = 0.05;  // m/s, before geometry/safety caps
  double speed_pid_derivative_filter = 0.10;  // seconds, derivative on measurement
  double angular_acceleration_fraction = 0.75;  // reserve feedback authority
  double gain_schedule_min_speed = 0.05;
  std::size_t gain_schedule_samples = 12;
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
  double reference_curvature = 0.0, feedforward_angular = 0.0;
  std::size_t next_stop_index = 0;
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
  double reference_curvature = 0.0;
  double feedforward_angular = 0.0;
  Point2D nearest{0.0, 0.0};
  Point2D lookahead{0.0, 0.0};
  TrackerDiagnostics diagnostics;
};

// ROS-independent, forward-only tracker. Explicit profiles stop only at flagged points;
// the legacy XY-only API conservatively stops at every real bend.
// TRACKING_ERROR and GOAL_REACHED latch zero until a new path. External data freshness is the
// wrapper's responsibility; actual velocity closes the longitudinal PID and confirms stops.
class LqrTracker
{
public:
  explicit LqrTracker(TrackerConfig config);
  void set_path(const std::vector<Point2D> & path);  // invalid input clears the old path, then throws
  // Explicit profile: ordinary arc vertices pass continuously; only flagged points stop.
  void set_profile(const std::vector<ReferencePoint> & path, bool continuous_curvature = false);
  void clear_path();
  void reset_speed_control();  // also used by wrapper watchdog/data-freshness stops
  bool has_path() const {return !points_.empty();}
  const std::array<double, 2> & gain() const {return gain_;}
  TrackerResult step(Pose2D pose, Velocity2D actual);

private:
  struct Segment {Point2D start; Point2D end; double length; double yaw; double start_s;};
  struct Sample {Point2D point; double yaw, curvature; std::size_t segment;};
  Sample reference_at(double s) const;
  TrackerResult step_continuous(Pose2D pose, Velocity2D actual);
  TrackerResult step_legacy(Pose2D pose, Velocity2D actual);
  double speed_control(
    double target, double actual, double limit, double acceleration, double deceleration);
  TrackerConfig config_;
  std::array<double, 2> gain_;
  std::vector<std::array<double, 2>> scheduled_gains_;
  std::vector<Point2D> points_;
  std::vector<Segment> segments_;
  std::size_t segment_ = 0;
  double progress_ = 0.0;
  bool aligning_ = true;
  bool fault_ = false;
  bool reached_ = false;
  TrackerDiagnostics fault_diagnostics_;  // retain the FIRST fault until clear_path
  bool continuous_ = false;
  std::vector<ReferencePoint> reference_;
  std::vector<double> arc_lengths_, speed_limits_;
  std::size_t phase_begin_ = 0, next_stop_ = 0;
  double continuous_progress_ = 0.0;
  bool continuous_curvature_ = false;
  std::vector<double> acceleration_limits_, deceleration_limits_;
  std::vector<double> segment_speed_limits_;
  double speed_integral_ = 0.0, speed_derivative_ = 0.0, previous_speed_ = 0.0;
  double previous_command_ = 0.0, previous_reference_ = 0.0;
  bool speed_initialized_ = false;
};
}  // namespace controller
#endif  // CONTROLLER__LQR_TRACKER_HPP_

// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef CONTROLLER__SAFETY_GATE_HPP_
#define CONTROLLER__SAFETY_GATE_HPP_

#include <limits>
#include <vector>

namespace controller
{
struct SafetyConfig
{
  double forward_sector_half_angle;
  double stop_distance;
  double release_distance;
  double max_beam_gap;
  bool allow_positive_infinity;
};

// Angles are scan-frame angles; scan_yaw rotates ray directions into base_frame.
// Distances remain ranges measured from the LiDAR origin, not bumper clearance.
struct SafetyScan
{
  double angle_min;
  double angle_max;
  double angle_increment;
  double range_min;
  double range_max;
  double scan_yaw;
  std::vector<float> ranges;
};

struct ScanAssessment
{
  bool valid = false;
  double minimum_range = std::numeric_limits<double>::infinity();
};

ScanAssessment assess_scan(const SafetyConfig & config, const SafetyScan & scan);

// Independent of ROS and LQR. Failures/startup latch blocked until a valid scan
// reaches release_distance. Stale inputs must be reported by the ROS wrapper.
class SafetyGate
{
public:
  explicit SafetyGate(SafetyConfig config);
  ScanAssessment update(const SafetyScan & scan);
  void invalidate() {blocked_ = true;}
  bool blocked() const {return blocked_;}

private:
  SafetyConfig config_;
  bool blocked_ = true;
};
}  // namespace controller
#endif  // CONTROLLER__SAFETY_GATE_HPP_

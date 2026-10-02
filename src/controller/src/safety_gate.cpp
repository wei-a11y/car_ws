// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/safety_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

void validate(const SafetyConfig & config)
{
  if (!std::isfinite(config.forward_sector_half_angle) ||
    config.forward_sector_half_angle <= 0.0 || config.forward_sector_half_angle >= kPi / 2.0 ||
    !std::isfinite(config.stop_distance) || config.stop_distance <= 0.0 ||
    !std::isfinite(config.release_distance) || config.release_distance <= config.stop_distance ||
    !std::isfinite(config.max_beam_gap) || config.max_beam_gap <= 0.0 ||
    config.max_beam_gap > config.forward_sector_half_angle)
  {
    throw std::invalid_argument("Invalid forward sector, distances or beam coverage limit");
  }
}
}  // namespace

ScanAssessment assess_scan(const SafetyConfig & config, const SafetyScan & scan)
{
  validate(config);
  ScanAssessment result;
  if (scan.ranges.size() < 2 || !std::isfinite(scan.angle_min) ||
    !std::isfinite(scan.angle_max) || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment <= 0.0 || scan.angle_increment > config.max_beam_gap ||
    !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) ||
    scan.range_min < 0.0 || scan.range_max <= scan.range_min ||
    scan.range_max < config.release_distance || !std::isfinite(scan.scan_yaw))
  {
    return result;
  }
  const double span = scan.angle_increment * static_cast<double>(scan.ranges.size() - 1);
  const double end = scan.angle_min + span;
  // LaserScan metadata/count must agree, allowing only floating-point rounding.
  constexpr double kMetadataTolerance = 1e-4;
  if (!std::isfinite(end) || span > 2.0 * kPi + kMetadataTolerance ||
    std::abs(end - scan.angle_max) > kMetadataTolerance)
  {
    return result;
  }
  std::vector<double> angles;
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double angle = std::remainder(
      scan.angle_min + static_cast<double>(i) * scan.angle_increment + scan.scan_yaw, 2.0 * kPi);
    if (std::abs(angle) > config.forward_sector_half_angle) {
      continue;
    }
    angles.push_back(angle);
    const double range = scan.ranges[i];
    if (std::isinf(range) && range > 0.0 && config.allow_positive_infinity) {
      // Explicit simulation convention: +Inf means no return up to range_max.
      result.minimum_range = std::min(result.minimum_range, scan.range_max);
    } else if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      return ScanAssessment{};  // A hole in the safety sector is not free space.
    } else {
      result.minimum_range = std::min(result.minimum_range, range);
    }
  }
  if (angles.empty()) {
    return ScanAssessment{};
  }
  std::sort(angles.begin(), angles.end());
  if (angles.front() + config.forward_sector_half_angle > config.max_beam_gap ||
    config.forward_sector_half_angle - angles.back() > config.max_beam_gap)
  {
    return ScanAssessment{};
  }
  for (std::size_t i = 1; i < angles.size(); ++i) {
    if (angles[i] - angles[i - 1] > config.max_beam_gap) {
      return ScanAssessment{};
    }
  }
  result.valid = true;
  return result;
}

SafetyGate::SafetyGate(SafetyConfig config)
: config_(config)
{
  validate(config_);
}

ScanAssessment SafetyGate::update(const SafetyScan & scan)
{
  const auto result = assess_scan(config_, scan);
  if (!result.valid || result.minimum_range <= config_.stop_distance) {
    blocked_ = true;
  } else if (result.minimum_range >= config_.release_distance) {
    blocked_ = false;
  }
  return result;
}
}  // namespace controller

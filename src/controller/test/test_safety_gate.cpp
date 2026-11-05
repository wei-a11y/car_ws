// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/safety_gate.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

namespace
{
controller::SafetyConfig config()
{
  return {0.7853981633974483, 0.50, 0.65, 0.035, true};
}

controller::SafetyScan scan(float range = 8.0F)
{
  // Actual repository geometry: laser_joint yaw=3.14, sensor angles .349..5.934.
  return {0.349, 5.934, (5.934 - 0.349) / 359.0, 0.12, 8.0, 3.14,
    std::vector<float>(360, range)};
}

TEST(SafetyGate, RepositoryLaserRotationAndRearSector)
{
  auto message = scan();
  message.ranges[0] = 0.2F;  // Near the rear, not the forward safety sector.
  controller::SafetyGate gate(config());
  EXPECT_TRUE(gate.blocked());
  EXPECT_TRUE(gate.update(message).valid);
  EXPECT_FALSE(gate.blocked());
  message.ranges[180] = 0.4F;
  EXPECT_NEAR(gate.update(message).minimum_range, 0.4, 1e-6);
  EXPECT_TRUE(gate.blocked());
}

TEST(SafetyGate, StartupAndHysteresis)
{
  controller::SafetyGate gate(config());
  gate.update(scan(0.60F));
  EXPECT_TRUE(gate.blocked());
  gate.update(scan(0.70F));
  EXPECT_FALSE(gate.blocked());
  gate.update(scan(0.55F));
  EXPECT_FALSE(gate.blocked());
  gate.update(scan(0.50F));
  EXPECT_TRUE(gate.blocked());
  gate.update(scan(0.55F));
  EXPECT_TRUE(gate.blocked());
  gate.update(scan(0.70F));
  EXPECT_FALSE(gate.blocked());
}

TEST(SafetyGate, ExternalTimeoutRequiresReleaseAgain)
{
  controller::SafetyGate gate(config());
  gate.update(scan());
  gate.invalidate();
  gate.update(scan(0.60F));
  EXPECT_TRUE(gate.blocked());
  gate.update(scan());
  EXPECT_FALSE(gate.blocked());
}

TEST(SafetyGate, PositiveInfinityIsExplicitNoReturnConvention)
{
  controller::SafetyGate gate(config());
  const auto message = scan(std::numeric_limits<float>::infinity());
  EXPECT_TRUE(gate.update(message).valid);
  EXPECT_FALSE(gate.blocked());
  auto strict = config();
  strict.allow_positive_infinity = false;
  EXPECT_FALSE(controller::assess_scan(strict, message).valid);
}

TEST(SafetyGate, AnyInvalidForwardBeamStops)
{
  for (const float value : {std::numeric_limits<float>::quiet_NaN(),
      -std::numeric_limits<float>::infinity(), -1.0F, 0.10F, 8.1F})
  {
    controller::SafetyGate gate(config());
    gate.update(scan());
    auto message = scan();
    message.ranges[180] = value;
    EXPECT_FALSE(gate.update(message).valid);
    EXPECT_TRUE(gate.blocked());
  }
}

TEST(SafetyGate, InvalidRearBeamsDoNotInvalidateForwardCoverage)
{
  auto message = scan();
  message.ranges[0] = std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(controller::assess_scan(config(), message).valid);
}

TEST(SafetyGate, MissingSectorIsNotFreeSpace)
{
  auto message = scan();
  message.scan_yaw = 0.0;  // This scan geometry would leave a forward blind gap.
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
  message.angle_max = message.angle_min + 0.01;
  message.ranges.resize(2);
  message.angle_increment = 0.01;
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
}

TEST(SafetyGate, EmptyAndMalformedMetadata)
{
  auto message = scan();
  message.ranges.clear();
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
  message = scan();
  message.angle_increment = 0.0;
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
  message.angle_increment = -0.01;
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
  message = scan();
  message.angle_max += 0.1;
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
  message = scan();
  message.range_max = 0.6;
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
  message = scan();
  message.scan_yaw = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(controller::assess_scan(config(), message).valid);
}

TEST(SafetyGate, SectorAndDistancesAreConfigurable)
{
  auto changed = config();
  changed.forward_sector_half_angle = 0.4;
  changed.stop_distance = 0.8;
  changed.release_distance = 0.9;
  controller::SafetyGate gate(changed);
  gate.update(scan());
  EXPECT_FALSE(gate.blocked());
  gate.update(scan(0.75F));
  EXPECT_TRUE(gate.blocked());
}

TEST(SafetyGate, InvalidConfigurationIsRejected)
{
  auto changed = config();
  changed.release_distance = changed.stop_distance;
  EXPECT_THROW(controller::SafetyGate{changed}, std::invalid_argument);
  changed = config();
  changed.forward_sector_half_angle = 0.0;
  EXPECT_THROW(controller::SafetyGate{changed}, std::invalid_argument);
  changed = config();
  changed.max_beam_gap = 2.0;
  EXPECT_THROW(controller::SafetyGate{changed}, std::invalid_argument);
  changed = config();
  changed.stop_distance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(controller::SafetyGate{changed}, std::invalid_argument);
}

controller::SafetyConfig sparse_config()
{
  auto result = config();
  result.allow_positive_infinity = false;
  result.allow_sparse_positive_infinity = true;
  result.forward_sector_half_angle = 0.5;
  return result;
}

controller::SafetyScan forward_scan(float range = 8.0F)
{
  return {-0.5, 0.5, 0.01, 0.12, 8.0, 0.0, std::vector<float>(101, range)};
}

TEST(SafetyGate, SparseReturnsAreUnknownAndStrictModeStillRejectsThem)
{
  auto message = forward_scan(2.0F);
  for (std::size_t i = 20; i < 60; i += 2) {
    message.ranges[i] = std::numeric_limits<float>::infinity();
  }
  const auto result = controller::assess_scan(sparse_config(), message);
  ASSERT_TRUE(result.valid);
  EXPECT_STREQ(result.reason, "OK");
  EXPECT_EQ(result.sector_beams, 101U);
  EXPECT_EQ(result.valid_beams, 81U);
  EXPECT_DOUBLE_EQ(result.valid_fraction, 81.0 / 101.0);
  EXPECT_NEAR(result.max_valid_gap, 0.02, 1e-12);
  EXPECT_DOUBLE_EQ(result.minimum_range, 2.0);
  auto strict = sparse_config();
  strict.allow_sparse_positive_infinity = false;
  EXPECT_FALSE(controller::assess_scan(strict, message).valid);
}

TEST(SafetyGate, SparseValidFractionBoundaryAndTooManyMissingReturns)
{
  auto message = forward_scan();
  for (std::size_t i = 20; i < 60; i += 2) {
    message.ranges[i] = std::numeric_limits<float>::infinity();
  }
  auto settings = sparse_config();
  settings.min_valid_fraction = 81.0 / 101.0;
  EXPECT_TRUE(controller::assess_scan(settings, message).valid);
  settings.min_valid_fraction = std::nextafter(settings.min_valid_fraction, 1.0);
  EXPECT_STREQ(controller::assess_scan(settings, message).reason, "VALID_FRACTION");
  message.ranges[60] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(controller::assess_scan(sparse_config(), message).valid);
}

TEST(SafetyGate, SparseGapBoundaryAndBothSectorEdges)
{
  for (const std::size_t first : {0U, 40U, 91U}) {
    auto message = forward_scan();
    for (std::size_t i = first; i < first + 10; ++i) {
      message.ranges[i] = std::numeric_limits<float>::infinity();
    }
    auto settings = sparse_config();
    const auto result = controller::assess_scan(settings, message);
    EXPECT_GT(result.valid_fraction, settings.min_valid_fraction);
    // Internal ten-ray hole spans eleven increments; edge hole spans ten.
    EXPECT_NEAR(result.max_valid_gap, first == 40U ? 0.11 : 0.10, 1e-12);
    settings.max_valid_beam_gap = result.max_valid_gap;
    EXPECT_TRUE(controller::assess_scan(settings, message).valid);
    settings.max_valid_beam_gap = std::nextafter(result.max_valid_gap, 0.0);
    EXPECT_STREQ(controller::assess_scan(settings, message).reason, "VALID_BEAM_GAP");
  }
}

TEST(SafetyGate, SparseAllMissingAndOtherInvalidReturnsFailClosed)
{
  controller::SafetyGate gate(sparse_config());
  gate.update(forward_scan());
  EXPECT_FALSE(gate.blocked());
  auto result = gate.update(forward_scan(std::numeric_limits<float>::infinity()));
  EXPECT_FALSE(result.valid);
  EXPECT_STREQ(result.reason, "NO_FINITE_RETURN");
  EXPECT_EQ(result.valid_beams, 0U);
  EXPECT_TRUE(std::isinf(result.minimum_range));
  EXPECT_TRUE(gate.blocked());
  for (const float value : {std::numeric_limits<float>::quiet_NaN(),
      -std::numeric_limits<float>::infinity(), 0.0F, 0.1F, 8.1F})
  {
    auto message = forward_scan();
    message.ranges[40] = std::numeric_limits<float>::infinity();
    message.ranges[50] = value;
    result = gate.update(message);
    EXPECT_FALSE(result.valid);
    EXPECT_STREQ(result.reason, "INVALID_BEAM");
    EXPECT_TRUE(gate.blocked());
  }
}

TEST(SafetyGate, SparseNearObstacleAndInvalidScanKeepReleaseHysteresis)
{
  controller::SafetyGate gate(sparse_config());
  auto message = forward_scan();
  message.ranges[40] = std::numeric_limits<float>::infinity();
  gate.update(message);
  EXPECT_FALSE(gate.blocked());
  message.ranges[50] = 0.4F;
  ASSERT_TRUE(gate.update(message).valid);
  EXPECT_TRUE(gate.blocked());
  message.ranges[50] = 0.6F;
  gate.update(message);
  EXPECT_TRUE(gate.blocked());
  message.ranges[50] = 0.7F;
  gate.update(message);
  EXPECT_FALSE(gate.blocked());
  gate.update(forward_scan(std::numeric_limits<float>::infinity()));
  message.ranges[50] = 0.6F;
  gate.update(message);
  EXPECT_TRUE(gate.blocked());
  message.ranges[50] = 0.7F;
  gate.update(message);
  EXPECT_FALSE(gate.blocked());
}

TEST(SafetyGate, SparseModeRetainsRawGeometryChecks)
{
  auto message = forward_scan();
  message.angle_max += 0.1;
  EXPECT_STREQ(controller::assess_scan(sparse_config(), message).reason, "INVALID_METADATA");
  message = forward_scan();
  message.scan_yaw = 0.1;  // Raw lattice no longer covers the left sector edge.
  EXPECT_STREQ(controller::assess_scan(sparse_config(), message).reason, "SECTOR_COVERAGE");
}

TEST(SafetyGate, SparseGapAcrossScanArrayBoundaryIsNotLost)
{
  constexpr double pi = 3.14159265358979323846;
  auto message = forward_scan();
  message.angle_min = -pi;
  message.angle_increment = 2.0 * pi / 720.0;
  message.angle_max = message.angle_min + 719.0 * message.angle_increment;
  message.scan_yaw = pi;
  message.ranges.assign(720, 8.0F);
  for (std::size_t i = 0; i < 7; ++i) {
    message.ranges[i] = std::numeric_limits<float>::infinity();
    message.ranges[719 - i] = std::numeric_limits<float>::infinity();
  }
  const auto result = controller::assess_scan(sparse_config(), message);
  EXPECT_GT(result.valid_fraction, 0.8);
  EXPECT_NEAR(result.max_valid_gap, 15.0 * message.angle_increment, 1e-12);
  EXPECT_STREQ(result.reason, "VALID_BEAM_GAP");
}

TEST(SafetyGate, SparseConfigurationRejectsConflictsAndInvalidThresholds)
{
  auto settings = sparse_config();
  settings.allow_positive_infinity = true;
  EXPECT_THROW(controller::SafetyGate{settings}, std::invalid_argument);
  for (const double value : {0.0, -0.1, 1.01, std::numeric_limits<double>::quiet_NaN()}) {
    settings = sparse_config();
    settings.min_valid_fraction = value;
    EXPECT_THROW(controller::SafetyGate{settings}, std::invalid_argument);
  }
  for (const double value : {0.0, -0.1, 1.01, std::numeric_limits<double>::infinity()}) {
    settings = sparse_config();
    settings.max_valid_beam_gap = value;
    EXPECT_THROW(controller::SafetyGate{settings}, std::invalid_argument);
  }
}

TEST(SafetyGate, CapturedRealScanRejectsStrictAndPassesBoundedSparsePolicy)
{
  std::ifstream input(std::string(SAFETY_SCAN_FIXTURE_DIR) + "/real_lidar.scan");
  ASSERT_TRUE(input.is_open());
  controller::SafetyScan message;
  ASSERT_TRUE(
    static_cast<bool>(input >> message.angle_min >> message.angle_max >>
    message.angle_increment >> message.range_min >> message.range_max >> message.scan_yaw));
  std::string token;
  while (input >> token) {
    message.ranges.push_back(std::stof(token));
  }
  ASSERT_EQ(message.ranges.size(), 720U);
  auto settings = sparse_config();
  settings.forward_sector_half_angle = config().forward_sector_half_angle;
  auto strict = settings;
  strict.allow_sparse_positive_infinity = false;
  EXPECT_FALSE(controller::assess_scan(strict, message).valid);
  const auto result = controller::assess_scan(settings, message);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.sector_beams, 179U);
  EXPECT_EQ(result.valid_beams, 147U);
  EXPECT_NEAR(result.valid_fraction, 147.0 / 179.0, 1e-12);
  EXPECT_NEAR(result.max_valid_gap, 0.08714509196579456, 1e-9);
  EXPECT_NEAR(result.minimum_range, 2.026, 1e-5);
}
}  // namespace

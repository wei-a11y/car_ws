// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "controller/safety_gate.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

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
}  // namespace

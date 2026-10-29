// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAN__GRID_FINGERPRINT_HPP_
#define PLAN__GRID_FINGERPRINT_HPP_
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <string>
#include "plan/planning_grid.hpp"
namespace plan
{
// Stable little-endian FNV-1a64 for matching a diagnostic map snapshot, not cryptography.
inline std::string grid_fingerprint(const PlanningGrid & grid)
{
  std::uint64_t hash = 14695981039346656037ULL;
  const auto byte = [&](std::uint8_t b) {hash ^= b;hash *= 1099511628211ULL;};
  const auto integer = [&](std::uint64_t value) {
      for (int i = 0; i < 8; ++i) {
        byte((value >> (8 * i)) & 255);
      }
    };
  const auto real = [&](double value) {
      std::uint64_t bits;std::memcpy(&bits, &value, sizeof(bits));integer(bits);
    };
  const auto & g = grid.geometry();integer(g.width);integer(g.height);
  real(g.resolution);real(g.origin_x);real(g.origin_y);real(g.origin_yaw);
  for (const auto value:grid.inflated_grid()) {
    byte(static_cast<std::uint8_t>(value));
  }
  std::ostringstream out;out << std::hex << std::setw(16) << std::setfill('0') << hash;
  return out.str();
}
}  // namespace plan
#endif  // PLAN__GRID_FINGERPRINT_HPP_

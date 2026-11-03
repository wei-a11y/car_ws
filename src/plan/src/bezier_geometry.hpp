// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef BEZIER_GEOMETRY_HPP_
#define BEZIER_GEOMETRY_HPP_
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include "plan/path_processing.hpp"
namespace plan
{namespace bezier
{
const double kPi = std::acos(-1.0);
inline Point2D add(Point2D a, Point2D b) {return {a.x + b.x, a.y + b.y};}
inline Point2D sub(Point2D a, Point2D b) {return {a.x - b.x, a.y - b.y};}
inline Point2D mul(Point2D a, double s) {return {a.x * s, a.y * s};}
inline double dot(Point2D a, Point2D b) {return a.x * b.x + a.y * b.y;}
inline double cross(Point2D a, Point2D b) {return a.x * b.y - a.y * b.x;}
inline double norm(Point2D a) {return std::hypot(a.x, a.y);}
inline double dist(Point2D a, Point2D b) {return norm(sub(a, b));}
inline Point2D direction(double yaw) {return {std::cos(yaw), std::sin(yaw)};}
inline double heading(Point2D a, Point2D b) {return std::atan2(b.y - a.y, b.x - a.x);}
inline double wrap(double a) {return std::remainder(a, 2.0 * kPi);}

using Bezier = std::array<Point2D, 6>;
inline Bezier hermite(
  Point2D a, double yaw_a, double ka, Point2D b, double yaw_b, double kb,
  double h)
{
  const auto da = mul(direction(yaw_a), h), db = mul(direction(yaw_b), h);
  const auto dda = mul(direction(yaw_a + kPi / 2), h * h * ka);
  const auto ddb = mul(direction(yaw_b + kPi / 2), h * h * kb);
  return {a, add(a, mul(da, .2)), add(add(a, mul(da, .4)), mul(dda, .05)),
    add(sub(b, mul(db, .4)), mul(ddb, .05)), sub(b, mul(db, .2)), b};
}

inline std::pair<Bezier, Bezier> split(const Bezier & b)
{
  Bezier left{}, right{}, work = b;
  left[0] = b[0]; right[5] = b[5];
  for (std::size_t j = 1; j < 6; ++j) {
    for (std::size_t i = 0; i < 6 - j; ++i) {
      work[i] = mul(add(work[i], work[i + 1]), .5);
    }
    left[j] = work[0]; right[5 - j] = work[5 - j];
  }
  return {left, right};
}

inline double choose(unsigned n, unsigned k)
{
  // All products used by the quintic certificates have degree <= 8. These small
  // integer binomials are exactly representable and avoid repeated inner loops.
  static const std::array<std::array<double, 9>, 9> table = []() {
      std::array<std::array<double, 9>, 9> values{};
      for (std::size_t i = 0; i < values.size(); ++i) {
        values[i][0] = values[i][i] = 1;
        for (std::size_t j = 1; j < i; ++j) {
          values[i][j] = values[i - 1][j - 1] + values[i - 1][j];
        }
      }
      return values;
    }();
  if (n < table.size() && k <= n) {return table[n][k];}
  double r = 1.0;
  for (unsigned i = 1; i <= k; ++i) {
    r *= static_cast<double>(n + 1 - i) / i;
  }
  return r;
}

// Bernstein coefficients bound products on the ENTIRE interval, not only samples.
template<typename A, typename B>
inline double product_bound(
  const A & a, const B & b,
  bool use_dot)
{
  const auto n = static_cast<unsigned>(a.size() - 1), m = static_cast<unsigned>(b.size() - 1);
  double bound = 0.0;
  for (unsigned k = 0; k <= n + m; ++k) {
    double coefficient = 0.0;
    for (unsigned i = 0; i <= n; ++i) {
      if (k < i || k - i > m) {continue;}
      coefficient += choose(n, i) * choose(m, k - i) / choose(n + m, k) *
        (use_dot ? dot(a[i], b[k - i]) : cross(a[i], b[k - i]));
    }
    bound = std::max(bound, std::abs(coefficient));
  }
  return bound;
}

inline std::vector<Point2D> derivative(const std::vector<Point2D> & p)
{
  std::vector<Point2D> d;
  for (std::size_t i = 1; i < p.size(); ++i) {
    d.push_back(mul(sub(p[i], p[i - 1]), p.size() - 1));
  }
  return d;
}

inline Bezier from_knots(const double * a, const double * b)
{
  const auto da = mul(direction(a[2]), a[3]), db = mul(direction(b[2]), b[3]);
  const auto dda = mul(direction(a[2] + kPi / 2), a[3] * a[3] * a[4]);
  const auto ddb = mul(direction(b[2] + kPi / 2), b[3] * b[3] * b[4]);
  const Point2D p{a[0], a[1]}, q{b[0], b[1]};
  return {p, add(p, mul(da, .2)), add(add(p, mul(da, .4)), mul(dda, .05)),
    add(sub(q, mul(db, .4)), mul(ddb, .05)), sub(q, mul(db, .2)), q};
}
inline Point2D value(std::vector<Point2D> p, double t)
{
  for (std::size_t n = p.size(); n > 1; --n) {
    for (std::size_t i = 0; i + 1 < n; ++i) {
      p[i] = add(mul(p[i], 1 - t), mul(p[i + 1], t));
    }
  }
  return p.front();
}
struct Geometry {Point2D point; double yaw, curvature, rate;};
template<std::size_t N>
inline std::array<Point2D, N - 1> array_derivative(const std::array<Point2D, N> & p)
{
  std::array<Point2D, N - 1> d;
  for (std::size_t i = 1; i < N; ++i) {
    d[i - 1] = mul(sub(p[i], p[i - 1]), N - 1);
  }
  return d;
}
template<std::size_t N>
inline Point2D array_value(std::array<Point2D, N> p, double t)
{
  for (std::size_t n = N; n > 1; --n) {
    for (std::size_t i = 0; i + 1 < n; ++i) {
      p[i] = add(mul(p[i], 1 - t), mul(p[i + 1], t));
    }
  }
  return p.front();
}
inline Geometry geometry(const Bezier & b, double t)
{
  const auto d1 = array_derivative(b);
  const auto d2 = array_derivative(d1);
  const auto d3 = array_derivative(d2);
  const auto v = array_value(d1, t), a = array_value(d2, t), j = array_value(d3, t);
  const double n = std::max(1e-12, norm(v)), c = cross(v, a);
  return {array_value(b, t), std::atan2(v.y, v.x), c / std::pow(n, 3),
    cross(v, j) / std::pow(n, 4) - 3 * c * dot(v, a) / std::pow(n, 6)};
}
struct Bounds {double lower, curvature, rate, envelope;};
template<typename D1, typename D2, typename D3>
inline Bounds derivative_bounds(
  const Bezier & b, const D1 & d1, const D2 & d2, const D3 & d3, double derivative_floor)
{
  const auto chord = sub(b.back(), b.front());
  const double length = norm(chord);
  double lower = INFINITY, envelope = 0;
  for (const auto & p : d1) {
    lower = std::min(lower, length > 0 ? dot(p, chord) / length : 0.0);
  }
  for (const auto & p : b) {
    const double t = length > 0 ? std::clamp(
      dot(
        sub(
          p,
          b.front()), chord) / (length * length), 0.0, 1.0) : 0;
    envelope = std::max(envelope, dist(p, add(b.front(), mul(chord, t))));
  }
  const double c = product_bound(d1, d2, false);
  const double safe = std::max(lower, derivative_floor);
  return {lower, safe > 0 ? c / std::pow(safe, 3) : INFINITY,
    safe > 0 ? product_bound(d1, d3, false) / std::pow(safe, 4) +
    3 * c * product_bound(d1, d2, true) / std::pow(safe, 6) : INFINITY, envelope};
}
inline Bounds bounds(const Bezier & b, double derivative_floor = 0.0)
{
  const auto d1 = derivative(std::vector<Point2D>(b.begin(), b.end()));
  const auto d2 = derivative(d1), d3 = derivative(d2);
  return derivative_bounds(b, d1, d2, d3, derivative_floor);
}
inline Bounds fixed_bounds(const Bezier & b, double derivative_floor = 0.0)
{
  const auto d1 = array_derivative(b);
  const auto d2 = array_derivative(d1);
  const auto d3 = array_derivative(d2);
  return derivative_bounds(b, d1, d2, d3, derivative_floor);
}
inline std::pair<Bezier, Bezier> split_at(const Bezier & b, double t)
{
  Bezier left{}, right{}, work = b;
  left[0] = b[0];right[5] = b[5];
  for (std::size_t j = 1; j < 6; ++j) {
    for (std::size_t i = 0; i < 6 - j; ++i) {
      work[i] = add(mul(work[i], 1 - t), mul(work[i + 1], t));
    }
    left[j] = work[0];right[5 - j] = work[5 - j];
  }
  return {left, right};
}
inline Bezier interval(const Bezier & b, double from, double to)
{
  const auto prefix = split_at(b, to).first;
  return from == 0.0 ? prefix : split_at(prefix, from / to).second;
}
}}   // namespace plan::bezier
#endif  // BEZIER_GEOMETRY_HPP_

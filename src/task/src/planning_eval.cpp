// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
// Thin Python adapter: inflation, A* and supercover/path processing stay in plan.
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "plan/astar.hpp"
#include "plan/path_processing.hpp"
#include "plan/planning_grid.hpp"

namespace py = pybind11;
namespace
{
template<typename T>
T get(const py::dict & values, const char * key, T fallback)
{
  return values.contains(key) ? values[key].cast<T>() : fallback;
}

class Grid
{
public:
  Grid(const py::dict & geometry, const std::vector<int> & raw,
    const py::dict & settings, double extra_radius)
  {
    if (!std::isfinite(extra_radius) || extra_radius < 0.0) {
      throw std::invalid_argument("Extra carried-object radius must be finite and nonnegative");
    }
    plan::GridGeometry g{
      geometry["width"].cast<std::size_t>(), geometry["height"].cast<std::size_t>(),
      geometry["resolution"].cast<double>(), get(geometry, "origin_x", 0.0),
      get(geometry, "origin_y", 0.0), get(geometry, "origin_yaw", 0.0)};
    plan::GridConfig c{
      get(settings, "occupied_threshold", 65),
      // Recovery never regards unobserved map cells as safe manipulation space.
      true, get(settings, "robot_radius", 0.355) + extra_radius,
      get(settings, "safety_margin", 0.05),
      get(settings, "inflate_map_boundary", true),
      get(settings, "max_cells", std::size_t{4000000})};
    std::vector<std::int8_t> cells;
    cells.reserve(raw.size());
    for (int cell : raw) {
      if (cell < -1 || cell > 100) {
        throw std::invalid_argument("Occupancy must be -1 or 0..100");
      }
      cells.push_back(static_cast<std::int8_t>(cell));
    }
    {
      py::gil_scoped_release release;
      grid_ = std::make_unique<plan::PlanningGrid>(g, std::move(cells), c);
    }
    processing_ = {
      get(settings, "enable_shortcut", true), get(settings, "resample_spacing", 0.1),
      get(settings, "shortcut_max_lookahead", std::size_t{200}),
      get(settings, "max_output_points", std::size_t{100000})};
    plan::validate_path_processing_config(processing_);
  }

  py::dict route(double sx, double sy, double gx, double gy, double yaw) const
  {
    py::dict output;
    py::list path;
    output["path"] = path;
    output["length"] = py::float_(std::numeric_limits<double>::infinity());
    plan::Cell start{}, goal{};
    if (!std::isfinite(yaw) || !grid_->world_to_cell(sx, sy, start.x, start.y)) {
      output["status"] = "INVALID_START";
      return output;
    }
    if (!grid_->world_to_cell(gx, gy, goal.x, goal.y)) {
      output["status"] = "INVALID_GOAL";
      return output;
    }
    const auto result = [&]() {
        py::gil_scoped_release release;
        return plan::astar_search(grid_.get(), start, goal);
      }();
    output["status"] = plan::status_name(result.status);
    if (result.status != plan::PlanStatus::SUCCESS) {return output;}
    std::vector<plan::Point2D> raw;
    // Keep exact requested endpoints for manipulation reach validation. Every
    // connector is checked by the same conservative supercover as plan.
    raw.push_back({sx, sy});
    for (const auto & cell : result.cells) {
      const auto point = grid_->cell_to_world(cell.x, cell.y);
      if (std::hypot(raw.back().x - point.first, raw.back().y - point.second) > 1e-10) {
        raw.push_back({point.first, point.second});
      }
    }
    if (std::hypot(raw.back().x - gx, raw.back().y - gy) > 1e-10) {
      raw.push_back({gx, gy});
    }
    try {
      const auto processed = [&]() {
          py::gil_scoped_release release;
          return plan::process_path(*grid_, raw, processing_);
        }();
      double length = 0.0;
      for (std::size_t i = 0; i < processed.size(); ++i) {
        const auto & p = processed[i];
        path.append(py::make_tuple(p.x, p.y, i + 1 == processed.size() ? yaw : p.yaw));
        if (i) {length += std::hypot(p.x - processed[i - 1].x, p.y - processed[i - 1].y);}
      }
      output["length"] = length;
    } catch (const std::exception &) {
      output["status"] = "NO_PATH";
      output["path"] = py::list();
    }
    return output;
  }

  bool free(double x, double y) const
  {
    std::size_t cx = 0, cy = 0;
    return grid_->world_to_cell(x, y, cx, cy) && !grid_->is_blocked(cx, cy);
  }

  bool line_clear(double x0, double y0, double x1, double y1) const
  {
    return plan::line_of_sight(*grid_, {x0, y0}, {x1, y1});
  }

  std::vector<int> inflated() const
  {
    const auto & cells = grid_->inflated_grid();
    return std::vector<int>(cells.begin(), cells.end());
  }

private:
  std::unique_ptr<plan::PlanningGrid> grid_;
  plan::PathProcessingConfig processing_{};
};
}  // namespace

PYBIND11_MODULE(_planning_eval, module)
{
  module.doc() = "Recovery evaluation adapter using the repository plan C++ cores";
  py::class_<Grid>(module, "Grid")
  .def(py::init<const py::dict &, const std::vector<int> &, const py::dict &, double>(),
    py::arg("geometry"), py::arg("occupancy"), py::arg("config"),
    py::arg("extra_radius") = 0.0)
  .def("route", &Grid::route, py::arg("sx"), py::arg("sy"), py::arg("gx"),
    py::arg("gy"), py::arg("goal_yaw") = 0.0)
  .def("free", &Grid::free)
  .def("line_clear", &Grid::line_clear)
  .def_property_readonly("inflated", &Grid::inflated);
}

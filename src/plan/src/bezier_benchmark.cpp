// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "processing_parameters.hpp"
#include "path_experiments.hpp"
#include "path_geometry_optimizer.hpp"
#include "plan/astar.hpp"
#include "plan/grid_fingerprint.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rclcpp::Node>("planner");
    const auto config = plan::load_processing_config(*node);
    const auto input = node->declare_parameter<std::string>("benchmark_input_file");
    const auto output = node->declare_parameter<std::string>("benchmark_output_dir");
    const auto preserve = node->declare_parameter<bool>("benchmark_preserve_polyline", false);
    const auto p1 = node->declare_parameter<bool>("benchmark_p1", false);
    const auto named_variant = node->declare_parameter<std::string>("benchmark_variant", "");
    const auto inflated = node->declare_parameter<bool>("benchmark_input_inflated", false);
    const auto profile_input = node->declare_parameter<std::string>("benchmark_profile_input", "");
    const auto configured = node->declare_parameter<bool>("benchmark_use_config_switches", false);
    const auto speed_only = node->declare_parameter<bool>("benchmark_speed_only", false);
    if (speed_only && (!configured || profile_input.empty())) {
      throw std::invalid_argument("Speed replay requires configured switches and a frozen profile");
    }
    std::ifstream stream(input);
    plan::GridGeometry geometry{};plan::GridConfig grid_config{};
    stream >> geometry.width >> geometry.height >> geometry.resolution >> geometry.origin_x >>
    geometry.origin_y >> geometry.origin_yaw;
    stream >> grid_config.occupied_threshold >> grid_config.unknown_is_obstacle >>
    grid_config.robot_radius >>
    grid_config.safety_margin >> grid_config.inflate_map_boundary >> grid_config.max_cells;
    if (!stream || geometry.width == 0 || geometry.height == 0 ||
      geometry.width > grid_config.max_cells / geometry.height)
    {
      throw std::runtime_error("Invalid benchmark grid");
    }
    std::vector<std::int8_t> cells(geometry.width * geometry.height);
    for (auto & cell:cells) {
      int value;stream >> value;cell = static_cast<std::int8_t>(value);
    }
    const auto grid = inflated ? plan::PlanningGrid::from_inflated_grid(
      geometry, cells, grid_config.max_cells) : plan::PlanningGrid(geometry, cells, grid_config);
    std::vector<plan::PathPoint2D> frozen_profile;
    if (!profile_input.empty()) {
      std::ifstream profile(profile_input);std::string line;std::getline(profile, line);
      while (std::getline(profile, line)) {
        std::replace(line.begin(), line.end(), ',', ' ');std::istringstream row(line);
        std::size_t index;int stop, type;plan::PathPoint2D p;
        row >> index >> p.x >> p.y >> p.tangent_yaw >> p.curvature >> stop >> type >>
        p.speed_limit >> p.curvature_bound >> p.curvature_rate_bound;
        if (!row) {throw std::runtime_error("Invalid frozen profile");}
        p.yaw = p.tangent_yaw;p.stop_required = stop;p.segment_type = type;frozen_profile.push_back(
          p);
      }
      if (frozen_profile.size() < 2) {throw std::runtime_error("Missing frozen profile");}
    }
    std::size_t count;stream >> count;
    if (!stream || count > 100) {throw std::runtime_error("Invalid benchmark scenarios");}
    std::filesystem::create_directories(output);
    std::ofstream summary(output + "/offline.jsonl");summary.precision(12);
    for (std::size_t scenario = 0; scenario < count; ++scenario) {
      std::string name;std::size_t n;stream >> name >> n;
      std::vector<plan::Point2D> raw;
      if (n == 0) {
        double x, y, gx, gy;stream >> x >> y >> gx >> gy;
        plan::Cell start{}, goal{};
        if (!grid.world_to_cell(
            x, y, start.x,
            start.y) || !grid.world_to_cell(gx, gy, goal.x, goal.y))
        {
          throw std::runtime_error("Benchmark endpoint outside map");
        }
        const auto search = plan::astar_search(&grid, start, goal);
        if (search.status != plan::PlanStatus::SUCCESS) {
          throw std::runtime_error("Benchmark A* failed: " + name);
        }
        for (const auto cell:search.cells) {
          const auto p = grid.cell_to_world(cell.x, cell.y);raw.push_back({p.first, p.second});
        }
      } else {
        if (n > config.max_output_points) {throw std::runtime_error("Too many benchmark points");}
        for (std::size_t i = 0; i < n; ++i) {
          plan::Point2D p;stream >> p.x >> p.y;raw.push_back(p);
        }
      }
      if (!stream) {throw std::runtime_error("Truncated benchmark input");}
      if (p1) {
        std::ofstream raw_file(output + "/" + name + "_raw.csv");raw_file.precision(16);
        raw_file << "x,y\n";
        for (auto q:raw) {
          raw_file << q.x << ',' << q.y << '\n';
        }
      }
      const std::vector<unsigned> variants = !named_variant.empty() ?
        std::vector<unsigned>{named_variant == "baseline" ? 0U : 1U} :
      p1 ? std::vector<unsigned>{0U, 1U, 1U} :
      std::vector<unsigned>{0U, 1U, 2U, 4U, 3U, 7U};
      std::size_t variant_index = 0;
      for (unsigned flags:variants) {
        auto c = config;
        if (preserve) {c.enable_shortcut = false;}
        if (!configured) {
          c.enable_window_bezier = flags & 1;
          c.motion.enable_dynamic_angular_budget = flags & 2;
          c.enable_turn_speed_coordination = flags & 4;
        }
        const unsigned effective_flags = (c.enable_window_bezier ? 1U : 0U) |
          (c.motion.enable_dynamic_angular_budget ? 2U : 0U) |
          (c.enable_turn_speed_coordination ? 4U : 0U);
        if (speed_only) {c.enable_window_bezier = false;}
        const std::string variant =
          !named_variant.empty() ? named_variant : p1 ? std::vector<std::string>{"baseline",
          "legacy",
          "geometry_first"}[variant_index++] : std::to_string(flags);
        if (p1 && named_variant.empty()) {
          c.window_geometry_policy = variant == "geometry_first" ? "geometry_first" : "legacy";
        }
        const auto begun = std::chrono::steady_clock::now();plan::PathProcessingStats stats;
        const auto path = frozen_profile.empty() ? plan::process_path(grid, raw, c, &stats) :
          plan::process_experiments(grid, frozen_profile, c, stats);
        stats.final_geometry_id = plan::geometry_fingerprint(path);
        if (speed_only && stats.final_geometry_id != plan::geometry_fingerprint(frozen_profile)) {
          throw std::runtime_error("Speed replay changed frozen geometry");
        }
        stats.grid_fingerprint = plan::grid_fingerprint(grid);
        const double seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - begun).count();
        const auto envelope = plan::speed_envelope(plan::path_speed_segments(path), c.motion);
        summary << "{\"scenario\":\"" << name << "\",\"variant\":\"" << variant <<
          "\",\"flags\":" << effective_flags << ",\"speed_only\":" <<
          (speed_only ? "true" : "false") << ",\"seconds\":" <<
          seconds <<
          ",\"executable_time\":" <<
          plan::executable_time(
          path,
          c.motion) << ",\"report\":" << plan::processing_report(stats) << "}\n";
        summary.flush();
        std::ofstream csv(output + "/" + name + "_" + variant + ".csv");csv.precision(17);
        csv <<
          "index,x,y,yaw,curvature,stop_required,segment_type,speed_limit,curvature_bound,curvature_rate_bound\n";
        for (std::size_t i = 0; i < path.size(); ++i) {
          const auto & p = path[i];
          csv << i << ',' << p.x << ',' << p.y << ',' << p.tangent_yaw << ',' << p.curvature <<
            ',' << p.stop_required << ',' <<
            static_cast<unsigned>(p.segment_type) << ',' << p.speed_limit << ',' <<
            p.curvature_bound << ',' << p.curvature_rate_bound << '\n';
        }
        std::ofstream speed(output + "/" + name + "_" + variant + "_speed.csv");
        speed << "distance,speed\n";
        for (std::size_t i = 0; i < envelope.speed.size(); ++i) {
          speed << envelope.distance[i] << ',' << envelope.speed[i] << '\n';
        }
        std::ofstream limits(output + "/" + name + "_" + variant + "_limits.csv");
        limits.precision(17);
        limits << "index,K,Q,nominal_cap,angular_cap,lateral_cap,rate_cap,profile_cap,"
          "acceleration,deceleration,envelope_begin,envelope_end,limiter\n";
        const auto segments = plan::path_speed_segments(path);
        for (std::size_t i = 0; i < segments.size(); ++i) {
          const auto & edge = segments[i];
          const auto bound = plan::curve_limits(c.motion, edge.curvature, edge.rate);
          const double nominal = std::min(c.motion.v_nominal, c.motion.v_max);
          const double angular = edge.curvature > 0 ?
            c.motion.feedforward_angular_fraction * c.motion.w_max / edge.curvature : INFINITY;
          const double lateral = edge.curvature > 0 ?
            std::sqrt(c.motion.lateral_acceleration / edge.curvature) : INFINITY;
          const double budget = plan::angular_budget(c.motion) /
            (c.motion.enable_dynamic_angular_budget ? 1.0 / c.motion.dynamic_budget_utilization : 2.0);
          const double rate = edge.rate > 0 ? std::sqrt(budget / edge.rate) : INFINITY;
          const double cap = std::min({nominal, angular, lateral, rate, edge.cap});
          std::string reason;
          const auto append = [&](double value, const std::string & label) {
              if (std::abs(value - cap) <= 1e-10) {
                reason += (reason.empty() ? "" : "|") + label;
              }
            };
          append(nominal, "nominal");append(angular, "angular_velocity");
          append(lateral, "lateral_acceleration");append(rate, "curvature_rate");
          if (edge.cap + 1e-10 < std::min({nominal, angular, lateral, rate})) {
            reason += "coordination_or_input_cap";
          }
          limits << i << ',' << edge.curvature << ',' << edge.rate << ',' << nominal << ',' <<
            angular << ',' << lateral << ',' << rate << ',' << edge.cap << ',' <<
            bound.acceleration << ',' << bound.deceleration << ',' << envelope.speed[2 * i] <<
            ',' << envelope.speed[2 * i + 2] << ',' << reason << '\n';
        }
      }
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';result = 1;
  }
  rclcpp::shutdown();return result;
}

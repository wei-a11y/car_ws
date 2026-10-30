// Copyright 2026 car_ws contributors
// SPDX-License-Identifier: Apache-2.0
#include "processing_parameters.hpp"
#include "path_experiments.hpp"
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
        if (preserve) {c.enable_shortcut = false;} c.enable_window_bezier = flags & 1;
        c.motion.enable_dynamic_angular_budget = flags & 2;
        c.enable_turn_speed_coordination = flags & 4;
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
        stats.grid_fingerprint = plan::grid_fingerprint(grid);
        const double seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - begun).count();
        const auto envelope = plan::speed_envelope(plan::path_speed_segments(path), c.motion);
        summary << "{\"scenario\":\"" << name << "\",\"variant\":\"" << variant <<
          "\",\"flags\":" << flags << ",\"seconds\":" <<
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
      }
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';result = 1;
  }
  rclcpp::shutdown();return result;
}

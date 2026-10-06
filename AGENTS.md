# AGENTS.md — Project Rules (V3)

## Purpose and Scope

This file defines the long-term rules for Codex work in the `car_ws` repository. It is not a roadmap and must not be used to duplicate a phase-by-phase implementation plan.

- These rules apply to the entire repository unless a more specific `AGENTS.md` exists in a subdirectory.
- The supported development environment is Ubuntu 22.04 with ROS 2 Humble.
- Confirm the active branch with `git branch --show-current` before changes; do not assume a fixed development branch or switch branches without explicit instruction.
- Work must remain within this repository.

## Confirmed Project Context

- The V1 navigation baseline is a lightweight, self-developed pipeline:
  `Goal -> A* -> Path Processing -> LQR -> Safety Gate -> differential-drive base`.
- The Gazebo differential-drive base can already be driven by keyboard teleoperation.
- A single-line 2D LiDAR Gazebo simulation plugin already exists.
- The URDF already reserves a link for a depth camera.
- No Gazebo depth-camera sensor/plugin has been added yet.
- The current primary modules are `task`, `plan`, `controller`, and `common_msg`.
- The later research direction is:
  `navigation failure -> movable obstacle perception -> environment interaction -> replan`.

## Architecture and Interface Rules

- Prefer standard ROS 2 messages wherever they satisfy the interface requirements.
- The `plan -> controller` interface must use `nav_msgs/Path`.
- Do not create a custom `Path.msg` without a concrete, documented reason that standard messages cannot meet.
- `task` must not publish or otherwise supply the robot's real-time start pose.
- The current robot pose must come from TF, localization, or odometry, as confirmed by the repository's actual implementation.
- Goals use `geometry_msgs/PoseStamped`, maps/grids use `nav_msgs/OccupancyGrid`, and 2D LiDAR uses `sensor_msgs/LaserScan`.
- LQR publishes `geometry_msgs/Twist` on a separate raw-command topic. In autonomous navigation, Safety Gate is the final command stage before the base; do not remap LQR directly to the base command topic or introduce competing autonomous publishers there.
- Prefer parameters and remapping over hardcoded topic/frame names. The default interfaces listed below do not prohibit explicit deployment overrides.
- Keep core algorithms separate from ROS Node wrappers wherever practical so that algorithms remain independently testable and reusable.
- Put tunable parameters in YAML whenever practical; avoid unexplained magic numbers in source code.

## Scope and Dependency Guardrails

- Do not perform broad or speculative refactors of the existing URDF or Gazebo setup.
- Keep the reserved depth-camera link, but do not add a depth-camera Gazebo sensor/plugin during the navigation baseline phase.
- Do not introduce PCL, YOLO, or a complete 3D navigation stack prematurely.
- Do not introduce any MID360 dependency.
- Do not bring in the full Nav2 stack merely to implement the A* and LQR baseline.
- Existing selective use of `nav2_map_server`, `nav2_amcl`, and `nav2_lifecycle_manager` is part of the baseline, not authorization to replace the self-developed planning/control pipeline with full Nav2.
- Complete only the current Phase. Do not pre-implement later research phases.

## Working Procedure

- Read the relevant repository files before making a change.
- Do not modify files unrelated to the current task.
- Confirm uncertain interfaces, names, paths, frames, topics, parameters, and implementations from the repository; never guess them.
- After every change, provide the relevant build and test commands. Do not claim that a command passed unless it was actually run successfully.
- If a critical prerequisite is missing or inconsistent, stop expanding the modification scope and report the prerequisite and its impact.
- Each Phase should normally be delivered as an independent git commit.
- Preserve existing user changes and avoid destructive git operations unless explicitly requested.

## Confirmed Repository Facts

The following facts are confirmed from current repository source and configuration, including the original Phase 0 audit and subsequent navigation implementation. Configured behavior is not proof of runtime readiness or validated navigation safety.

- The Gazebo base command topic is `/cmd_vel`. The `gazebo_ros2_control` configuration remaps `/diff_drive_controller/cmd_vel_unstamped` to `/cmd_vel`, and the differential-drive controller is configured with `use_stamped_vel: false`.
- The differential-drive odometry topic is `/odom`, remapped from `/diff_drive_controller/odom`.
- The simulated 2D LiDAR publishes `sensor_msgs/LaserScan` on `/scan` with `frame_id` `laser_link`.
- The differential-drive controller uses `odom` as its odometry frame and `base_footprint` as its base frame, with odometry TF publishing enabled. This establishes the configured `odom -> base_footprint` TF edge.
- The URDF defines a fixed `base_footprint -> base_link` transform with a translation of `0.097 m` in z.
- The LiDAR link/frame is `laser_link`, attached to `base_link` by the fixed `laser_joint`.
- The reserved depth-camera link/frame is `camera_1_link`. The URDF also contains `camera_2_link`; both are fixed directly to `base_link`, and neither currently has a Gazebo sensor/plugin.
- The configured wheel radius is `0.1 m`, and the configured wheel separation is `0.286 m`.
- Gazebo drive uses `gazebo_ros2_control/GazeboSystem` through `libgazebo_ros2_control.so`, with `diff_drive_controller/DiffDriveController` commanding velocity interfaces for `L_wheel_joint` and `R_wheel_joint`.
- The main simulation launch entry is `src/urdf/launch/sim.launch.py`, installed by package `car_description` and invoked as `ros2 launch car_description sim.launch.py`. It starts Gazebo, `robot_state_publisher`, entity spawning, the joint-state broadcaster, the differential-drive controller, and RViz.
- The auxiliary model-display entry is `src/urdf/launch/display.launch.py`.
- The navigation entry is `src/urdf/launch/navigation.launch.py`, invoked as `ros2 launch car_description navigation.launch.py`. It includes simulation/map loading by default and starts AMCL, localization lifecycle management, planning-grid generation, A* with path post-processing, LQR, and Safety Gate. `start_sim:=false` requires an already running compatible simulation and map server.
- `sim.launch.py` now starts `nav2_map_server` and its lifecycle manager. Default source assets are `world_model/model.world` and `map/edited/edited.yaml`, installed by `car_description`; `world` and `map_yaml` launch arguments can override them. The default map YAML specifies `0.05 m` resolution.
- `src/urdf/config/amcl.yaml` configures AMCL to publish `map -> odom`, using `/map`, `/scan`, and `base_footprint`. Automatic initial-pose setup is disabled; supply a valid map-frame initial pose through RViz `/initialpose`. Do not assume that map coordinates equal Gazebo world coordinates or substitute an identity static transform for localization.
- Navigation deployment uses planning frame `map`. Planner start pose and LQR current pose come from TF; LQR also reads `nav_msgs/Odometry` on `/odom` for actual velocity and stop confirmation, not as a fallback pose source.
- The configured planning robot radius is `0.355 m`, a conservative rounded collision-envelope radius, in `src/plan/config/planning_grid.yaml`; the repository provides `src/plan/tools/verify_robot_radius.py`. Recheck the envelope if robot geometry changes rather than substituting wheel radius/separation for robot size.

### Default Navigation Interfaces

| Data | Default topic | Message |
| --- | --- | --- |
| Goal | `/goal_pose` | `geometry_msgs/PoseStamped` |
| Map | `/map` | `nav_msgs/OccupancyGrid` |
| Raw / inflated planning grids | `/plan/raw_grid`, `/plan/inflated_grid` | `nav_msgs/OccupancyGrid` |
| Raw / processed path | `/plan/raw_path`, `/plan` | `nav_msgs/Path` |
| LQR raw / base command | `/cmd_vel_raw`, `/cmd_vel` | `geometry_msgs/Twist` |
| Actual base velocity | `/odom` | `nav_msgs/Odometry` |
| 2D LiDAR | `/scan` | `sensor_msgs/LaserScan` |

The planner consumes the inflated grid, not the raw map directly. Raw and processed paths/grids remain available for RViz debugging. The interface-contract YAML in `common_msg` is not a node launcher; actual node YAML and launch overrides determine deployed parameters.

## Configuration and Tracking Semantics

- `navigation.launch.py` loads each planning/LQR node's own package YAML first, then `src/urdf/config/navigation_baseline.yaml` as deployment overrides. Sections are matched by node name; later matching values override earlier ones. Safety Gate uses its own YAML plus explicit launch overrides, not the navigation-baseline YAML in the current launch.
- When diagnosing or tuning, inspect the complete launch parameter chain and verify effective values with `ros2 param get`. Launch resolves installed package-share files, which may differ from edited source files until rebuilt or symlinked. Current controller parameters are read-only after startup; do not assume `ros2 param set` can apply tuning live.
- Keep tunable safety margins, gains, search distances, thresholds, and timeouts in YAML; do not freeze their current tuning values in this file or describe them as validated hardware limits.
- In the current tracker, nearest-point windows are along the current segment relative to recorded `progress_`, not a radius around the robot or its heading. Progress is reset on a new path or segment transition and is otherwise nondecreasing. Backward search does not enable reverse driving.
- `max_tracking_error` bounds Euclidean distance to the window-constrained nearest point, not just lateral error. The check also applies at startup and before alignment; the tracker does not provide arbitrary off-path recovery. A tracking fault latches zero output until a new accepted path resets it.
- The current lightweight tracker stops at segment corners and aligns before continuing. Preserve this behavior unless the requested task explicitly changes it; do not silently introduce continuous cornering or complex speed planning.

## Planning and Safety Guardrails

- Planning inflation uses the robot envelope radius plus a safety margin, with conservative occupied-cell geometry compensation. Treat this as a discretized map collision model, not a measured minimum clearance for every continuous path point or the actual robot trajectory.
- Safety Gate is an independent fail-closed forward-sector LiDAR stop gate with hysteresis, not a local planner, obstacle-avoidance controller, or complete swept-footprint collision checker. Its distances are ray ranges from the LiDAR origin, not bumper clearance or distance from the planning reference point.
- A map-feasible path can still be blocked by Safety Gate, including near corners or narrow passages. Compare coordinate references, LiDAR mounting, robot envelope, tracking/localization uncertainty, and braking behavior before reconciling thresholds; never equate planning inflation radius with LiDAR stop distance directly.
- In the current implementation, blocking publishes a full zero Twist, including suppression of in-place rotation. `enable=false` inhibits autonomous motion; it is not a safety bypass. Invalid/stale scan or raw input, TF/clock failures, and invalid/out-of-bound velocity commands must fail closed.
- Diagnose before loosening protections. Do not enlarge tracking tolerance, reduce safety distance, or bypass Safety Gate merely to hide a stop without evidence and an explicitly scoped tuning/change request. Preserve stop/release hysteresis and input watchdogs.
- Inspect `/cmd_vel_raw`, `/cmd_vel`, `/controller/lqr/status`, `/controller/safety/status`, and `/controller/safety/min_range` together. `RAW_ZERO` means the upstream command is zero; `CLEAR` only means the gate permits forwarding and does not imply motion. Use the LQR first-fault diagnostic snapshot to distinguish failed geometric guards from Path/TF/odometry/clock failures; Safety Gate does not report LQR's internal cause.

## Repository Facts Still To Be Confirmed

The following facts remain intentionally unspecified and must be confirmed from repository implementation or an explicit project decision before use:

- Actual geometric alignment between the selected occupancy map and Gazebo world, and the correct initial pose for a given run
- Measured localization/tracking accuracy, stopping distance, and validated planning/safety margins for simulation or hardware
- Runtime map/TF/input readiness and reliable closed-loop behavior for the current scenario; source configuration or isolated tests alone do not establish these

Conventional ROS names, Roadmap examples, generated build/install artifacts, and assumptions are not authoritative repository facts.

## Change Review Checklist

Before completing a task, verify that:

- The change belongs to the current Phase and requested scope.
- Relevant interfaces were confirmed from repository files.
- Standard ROS 2 messages were preferred and `nav_msgs/Path` is preserved for `plan -> controller`.
- Core algorithm logic and ROS Node integration remain appropriately separated.
- New tunable values are parameterized in YAML where practical.
- Effective deployment overrides were considered, and planning clearance was not confused with LiDAR ray distance or validated physical safety.
- The raw-command/Safety Gate separation and fail-closed behavior remain intact; a zero command was attributed using upstream/downstream status and diagnostics.
- No premature depth, perception, 3D navigation, MID360, or full Nav2 dependency was added.
- Unrelated files, especially URDF/Gazebo assets, were not broadly refactored.
- Relevant build and test commands are included in the handoff.

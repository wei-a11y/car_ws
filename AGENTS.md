# AGENTS.md — Project Rules (V2)

## Purpose and Scope

This file defines the long-term rules for Codex work in the `car_ws` repository. It is not a roadmap and must not be used to duplicate a phase-by-phase implementation plan.

- These rules apply to the entire repository unless a more specific `AGENTS.md` exists in a subdirectory.
- The supported development environment is Ubuntu 22.04 with ROS 2 Humble.
- The current development branch is `urdf_change`.
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
- Keep core algorithms separate from ROS Node wrappers wherever practical so that algorithms remain independently testable and reusable.
- Put tunable parameters in YAML whenever practical; avoid unexplained magic numbers in source code.

## Scope and Dependency Guardrails

- Do not perform broad or speculative refactors of the existing URDF or Gazebo setup.
- Keep the reserved depth-camera link, but do not add a depth-camera Gazebo sensor/plugin during the navigation baseline phase.
- Do not introduce PCL, YOLO, or a complete 3D navigation stack prematurely.
- Do not introduce any MID360 dependency.
- Do not bring in the full Nav2 stack merely to implement the A* and LQR baseline.
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

The following facts were confirmed from the tracked repository files during Phase 0 and may be treated as the current repository baseline:

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
- Map image/YAML assets exist under `map/`, but the tracked source currently contains no integrated map-server, SLAM, localization, or `map -> odom` publisher configuration or launch entry. The operator notes only show a manual external `slam_toolbox` launch command.

## Repository Facts Still To Be Confirmed

The following facts remain intentionally unspecified and must be confirmed from repository implementation or an explicit project decision before use:

- Map frame and the component responsible for publishing `map -> odom`
- Navigation-baseline localization implementation and authoritative current-pose source
- Which map asset, if any, will be loaded by the navigation baseline

Conventional ROS names, Roadmap examples, generated build/install artifacts, and assumptions are not authoritative repository facts.

## Change Review Checklist

Before completing a task, verify that:

- The change belongs to the current Phase and requested scope.
- Relevant interfaces were confirmed from repository files.
- Standard ROS 2 messages were preferred and `nav_msgs/Path` is preserved for `plan -> controller`.
- Core algorithm logic and ROS Node integration remain appropriately separated.
- New tunable values are parameterized in YAML where practical.
- No premature depth, perception, 3D navigation, MID360, or full Nav2 dependency was added.
- Unrelated files, especially URDF/Gazebo assets, were not broadly refactored.
- Relevant build and test commands are included in the handoff.

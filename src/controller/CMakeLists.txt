cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(common_msg REQUIRED)
find_package(plan REQUIRED)


# ============================================================
# LQR
# ============================================================

add_library(
  lqr_tracker_core
  src/lqr_tracker.cpp
  src/lqr_continuous.cpp
)
target_link_libraries(lqr_tracker_core PUBLIC plan::planning_grid_core)
ament_export_dependencies(plan)

target_compile_features(
  lqr_tracker_core
  PUBLIC
  cxx_std_17
)

target_include_directories(
  lqr_tracker_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  lqr_tracker_node
  src/lqr_tracker_node.cpp
)

target_link_libraries(
  lqr_tracker_node
  lqr_tracker_core
)

ament_target_dependencies(
  lqr_tracker_node
  rclcpp
  common_msg
  rcl_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# ============================================================
# DWA
# ============================================================

add_library(
  dwa_tracker_core
  src/dwa_tracker.cpp
)

target_compile_features(
  dwa_tracker_core
  PUBLIC
  cxx_std_17
)

target_include_directories(
  dwa_tracker_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  dwa_tracker_node
  src/dwa_tracker_node.cpp
)

target_link_libraries(
  dwa_tracker_node
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  dwa_tracker_core
)

ament_target_dependencies(
  dwa_tracker_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
)


# ============================================================
# Safety Gate
# ============================================================

add_library(
  safety_gate_core
  src/safety_gate.cpp
)

target_compile_features(
  safety_gate_core
  PUBLIC
  cxx_std_17
)

target_include_directories(
  safety_gate_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  safety_gate_node
  src/safety_gate_node.cpp
)

target_link_libraries(
  safety_gate_node
  safety_gate_core
)

ament_target_dependencies(
  safety_gate_node
  rclcpp
  rcl_interfaces
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS
    lqr_tracker_core
    dwa_tracker_core
    safety_gate_core

  EXPORT
    controller_targets

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    lqr_tracker_node
    dwa_tracker_node
    safety_gate_node

  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/record_path_tracking.py scripts/analyze_path_tracking.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/tracking_analysis.py DESTINATION lib/${PROJECT_NAME})

ament_export_targets(
  controller_targets
  HAS_LIBRARY_TARGET
)


# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_tracking_analysis test/test_tracking_analysis.py)
  ament_add_pytest_test(test_path_recorder test/test_path_recorder.py TIMEOUT 30
    ENV "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/recorder_ros_logs" "ROS_LOCALHOST_ONLY=1")

  ament_add_gtest(test_continuous_tracking test/test_continuous_tracking.cpp)
  target_link_libraries(test_continuous_tracking lqr_tracker_core plan::path_processing_core)
  ament_target_dependencies(test_continuous_tracking plan)

  ament_add_gtest(
    test_lqr_tracker
    test/test_lqr_tracker.cpp
  )

  target_link_libraries(
    test_lqr_tracker
    lqr_tracker_core
  )

  ament_add_gtest(
    test_safety_gate
    test/test_safety_gate.cpp
  )

  target_link_libraries(
    test_safety_gate
    safety_gate_core
  )

  ament_add_pytest_test(
    test_safety_gate_node
    test/test_safety_gate_node.py

    TIMEOUT 90

    ENV
      "GATE_EXECUTABLE=$<TARGET_FILE:safety_gate_node>"
      "GATE_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/safety_gate.yaml"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_LOCALHOST_ONLY=1"
  )

  ament_add_pytest_test(
    test_lqr_tracker_node
    test/test_lqr_tracker_node.py

    TIMEOUT 60

    ENV
      "TRACKER_EXECUTABLE=$<TARGET_FILE:lqr_tracker_node>"
      "TRACKER_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/lqr_tracker.yaml"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_LOCALHOST_ONLY=1"
  )

  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

endif()


ament_package()

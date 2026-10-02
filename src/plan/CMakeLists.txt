cmake_minimum_required(VERSION 3.8)
project(plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_library(planning_grid_core src/planning_grid.cpp)
target_compile_features(planning_grid_core PUBLIC cxx_std_17)
target_include_directories(planning_grid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(planning_grid_node src/planning_grid_node.cpp)
target_link_libraries(planning_grid_node planning_grid_core)
ament_target_dependencies(planning_grid_node rclcpp rcl_interfaces nav_msgs)

install(TARGETS planning_grid_core EXPORT planning_grid_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS planning_grid_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_export_targets(planning_grid_coreTargets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_planning_grid test/test_planning_grid.cpp)
  target_link_libraries(test_planning_grid planning_grid_core)
  ament_add_pytest_test(test_planning_grid_node test/test_planning_grid_node.py
    TIMEOUT 40
    ENV
      "PLANNING_GRID_EXECUTABLE=$<TARGET_FILE:planning_grid_node>"
      "PLANNING_GRID_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/planning_grid.yaml"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_LOCALHOST_ONLY=1"
  )
  add_test(NAME verify_robot_radius
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/verify_robot_radius.py
  )
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

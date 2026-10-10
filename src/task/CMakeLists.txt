cmake_minimum_required(VERSION 3.8)
project(task)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(plan REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
pybind11_add_module(_planning_eval src/planning_eval.cpp)
target_compile_features(_planning_eval PRIVATE cxx_std_17)
target_link_libraries(_planning_eval PRIVATE plan::planning_grid_core
  plan::astar_core plan::path_processing_core)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS _planning_eval LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})
ament_python_install_package(task_runtime)
install(PROGRAMS scripts/task_node scripts/recovery_task_node scripts/recovery_scenario
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md OBSTACLE_RECOVERY.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_task_core test/test_core.py
    ENV "PYTHONDONTWRITEBYTECODE=1" "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_task_node test/test_node.py TIMEOUT 120
    ENV "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}"
      "TASK_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_LOCALHOST_ONLY=1" "PYTHONDONTWRITEBYTECODE=1"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_task_pipeline test/test_pipeline.py TIMEOUT 75
    ENV "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}"
      "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs" "ROS_LOCALHOST_ONLY=1"
      "PYTHONDONTWRITEBYTECODE=1"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  foreach(recovery_test recovery_core gazebo_backend recovery_launch recovery_node)
    ament_add_pytest_test(test_task_${recovery_test} test/test_${recovery_test}.py
      TIMEOUT 150
      ENV "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}"
        "TASK_SOURCE=${CMAKE_CURRENT_SOURCE_DIR}"
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
        "ROS_LOCALHOST_ONLY=1" "PYTHONDONTWRITEBYTECODE=1"
      APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$<TARGET_FILE_DIR:_planning_eval>")
  endforeach()
  ament_add_pytest_test(test_task_recovery_pipeline test/test_recovery_pipeline.py TIMEOUT 480
    ENV "TMPDIR=${CMAKE_CURRENT_BINARY_DIR}" "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs"
      "ROS_LOCALHOST_ONLY=1" "PYTHONDONTWRITEBYTECODE=1"
    APPEND_ENV "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$<TARGET_FILE_DIR:_planning_eval>")
endif()
ament_package()

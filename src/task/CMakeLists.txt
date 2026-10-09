cmake_minimum_required(VERSION 3.8)
project(task)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(task_runtime)
install(PROGRAMS scripts/task_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
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
endif()
ament_package()

cmake_minimum_required(VERSION 3.8)
project(car_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# The edited PGM remains in the repository-level map archive. Install it next
# to the runtime YAML so map_server can resolve the relative image filename.
install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../map/edited/map.pgm
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()

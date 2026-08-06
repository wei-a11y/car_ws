cmake_minimum_required(VERSION 3.8)
project(car_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes textures urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Keep the existing repository-level Gazebo model as the single source of
# truth, while installing it as a regular Gazebo model for model:// lookup.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../world_model/
  DESTINATION share/${PROJECT_NAME}/models/navigation_world
)

ament_package()

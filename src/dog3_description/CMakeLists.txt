cmake_minimum_required(VERSION 3.8)
project(dog3_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch meshes urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

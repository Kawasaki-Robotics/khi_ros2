cmake_minimum_required(VERSION 3.5)
project(khi_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

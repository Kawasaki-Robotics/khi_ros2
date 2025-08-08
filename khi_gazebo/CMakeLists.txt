cmake_minimum_required(VERSION 3.5)
project(khi_gazebo)

find_package(ament_cmake REQUIRED)
find_package(khi_description REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

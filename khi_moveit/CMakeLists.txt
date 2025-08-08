cmake_minimum_required(VERSION 3.5)
project(khi_moveit)

find_package(ament_cmake REQUIRED)
find_package(moveit_common REQUIRED)
find_package(khi_description REQUIRED)
find_package(khi_msgs REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

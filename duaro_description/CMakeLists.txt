cmake_minimum_required(VERSION 3.8)
project(duaro_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config srdf launch scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install standalone scripts to lib so ros2 run works
install(PROGRAMS
  scripts/joint_state_reorder_duaro.py
  scripts/trajectory_bridge_duaro.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

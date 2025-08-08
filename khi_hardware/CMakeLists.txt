cmake_minimum_required(VERSION 3.5)
project(khi_hardware)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(
  TARGET_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  khi_msgs
)

# Find dependencies
find_package(ament_cmake REQUIRED)
foreach(DEPENDENCY IN ITEMS ${TARGET_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Find the directory containing KRNX library.
if( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*x86_64.*" )
  set(KRNX_LIBRARY_DIR_NAME x86_64)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*aarch64.*" )
  set(KRNX_LIBRARY_DIR_NAME aarch64)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*i386.*" )
  set(KRNX_LIBRARY_DIR_NAME i386)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*arm.*gnueabihf" )
  set(KRNX_LIBRARY_DIR_NAME armhf)
elseif( ${CMAKE_LIBRARY_ARCHITECTURE} MATCHES ".*arm.*gnueabi" )
  set(KRNX_LIBRARY_DIR_NAME armel)
else()
  message(WARNING "\nCould not detect system processor architecture: assuming 'x86_64'. This may result in non-working binaries.")
  set(KRNX_LIBRARY_DIR_NAME x86_64)
endif()

# Compile
add_library(
  ${PROJECT_NAME}
  SHARED
  src/khi_hardware_interface.cpp
  src/khi_driver.cpp
  src/khi_krnx_driver.cpp
  src/khi_mock_driver.cpp
  src/khi_service.cpp
  src/khi_publisher.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
find_library(
  krnx_LIBRARIES
  libkrnx.so
  PATHS lib/${KRNX_LIBRARY_DIR_NAME}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${krnx_LIBRARIES}
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${TARGET_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(hardware_interface khi_hardware.xml)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install KRNX libraries
get_filename_component(krnx_LIBRARIES_PATH ${krnx_LIBRARIES} DIRECTORY)
file(GLOB files_to_install "${krnx_LIBRARIES_PATH}/*")
message(STATUS "$krnx_LIBRARIES_PATH")
install(
  FILES ${files_to_install}
  DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${TARGET_DEPENDENCIES})

ament_package()

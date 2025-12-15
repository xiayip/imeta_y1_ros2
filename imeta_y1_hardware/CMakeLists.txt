cmake_minimum_required(VERSION 3.8)
project(imeta_y1_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(glog REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/imeta_y1_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  y1_sdk
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64")
  set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/y1_sdk/lib/arm64/liby1_sdk_arm64.so")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "AMD64")
  set(LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/y1_sdk/lib/x64/liby1_sdk_x64.so")
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_libraries(${PROJECT_NAME}
  ${LIB_PATH}
  glog::glog
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface imeta_y1_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()

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

add_library(${PROJECT_NAME} SHARED
  src/imeta_y1_hardware.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  y1_sdk
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(CPU_ARCH "x64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(CPU_ARCH "aarch64")
else()
  message(FATAL_ERROR "Unsupported CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
message(STATUS "CPU architecture: ${CPU_ARCH}")
find_library(SDK_LIB y1_sdk_${CPU_ARCH} HINTS y1_sdk/lib/${CPU_ARCH} REQUIRED)

target_link_libraries(${PROJECT_NAME}
  ${SDK_LIB}
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

cmake_minimum_required(VERSION 3.5)
project(ros_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(rclsrv_server src/rclsrv_server.cpp)
ament_target_dependencies(rclsrv_server rclcpp example_interfaces)

add_executable(rclsrv_client src/rclsrv_client.cpp)
ament_target_dependencies(rclsrv_client rclcpp example_interfaces)

install(TARGETS
  rclsrv_server
  rclsrv_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()

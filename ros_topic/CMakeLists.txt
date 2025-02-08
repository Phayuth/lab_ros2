cmake_minimum_required(VERSION 3.5)
project(ros_topic)

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
find_package(std_msgs REQUIRED)

add_executable(rclpub src/rclpub.cpp)
ament_target_dependencies(rclpub rclcpp std_msgs)

add_executable(rclsub src/rclsub.cpp)
ament_target_dependencies(rclsub rclcpp std_msgs)

add_executable(rclpub_num src/rclpub_num.cpp)
ament_target_dependencies(rclpub_num rclcpp std_msgs)

add_executable(rclsub_num src/rclsub_num.cpp)
ament_target_dependencies(rclsub_num rclcpp std_msgs)

install(TARGETS
  rclpub
  rclsub
  rclsub_num
  rclpub_num
  DESTINATION lib/${PROJECT_NAME})

ament_package()

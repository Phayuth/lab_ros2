cmake_minimum_required(VERSION 3.8)
project(ros_urdf_kdl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(rcutils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(kdl_robot src/kdl_robot.cpp)
ament_target_dependencies(kdl_robot kdl_parser urdf orocos_kdl Eigen3)

ament_package()

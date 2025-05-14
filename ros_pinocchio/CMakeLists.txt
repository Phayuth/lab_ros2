cmake_minimum_required(VERSION 3.8)
project(ros_pinocchio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)

add_executable(pinocchio_example src/pinocchio_example.cpp)
ament_target_dependencies(pinocchio_example ament_index_cpp Eigen3 eigen3_cmake_module)
target_link_libraries(pinocchio_example pinocchio::pinocchio)

install(
  TARGETS
  pinocchio_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

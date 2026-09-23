cmake_minimum_required(VERSION 3.8)
project(burger_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pick_place_node src/pick_place_node.cpp)
target_compile_features(pick_place_node PUBLIC cxx_std_17)
target_include_directories(pick_place_node PUBLIC ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(pick_place_node
  rclcpp
  moveit_task_constructor_core
  moveit_ros_planning_interface
  tf2_ros
  geometry_msgs
  moveit_msgs
  shape_msgs
)

install(TARGETS pick_place_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(burger_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf rviz visual vendor
  DESTINATION share/${PROJECT_NAME}
)

# Los URDF referencian package://burger_description/meshes/...; el enlace
# simbólico meshes -> visual/meshes no se instala por sí solo.
install(
  DIRECTORY visual/meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/scene_joint_relay.py scripts/scene_collision_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene_joint_relay test/test_scene_joint_relay.py)
  ament_add_pytest_test(test_scene_collision_markers test/test_scene_collision_markers.py)
endif()

ament_package()

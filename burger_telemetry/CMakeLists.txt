cmake_minimum_required(VERSION 3.8)
project(burger_telemetry)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.5)
project(nav2_bringup_extended)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY launch maps params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

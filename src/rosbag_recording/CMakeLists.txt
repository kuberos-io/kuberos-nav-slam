cmake_minimum_required(VERSION 3.8)
project(rosbag_recording)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

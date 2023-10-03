cmake_minimum_required(VERSION 3.8)
project(sim_data_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(diagnostic_updater REQUIRED)

# gazebo
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_executable(sim_data_publisher
  src/sim_data_publisher.cpp
)
ament_target_dependencies(sim_data_publisher
  geometry_msgs tf2 tf2_ros tf2_geometry_msgs
  ignition-gazebo6 ignition-transport11 ignition-msgs8
  diagnostic_updater
)

install(
  TARGETS sim_data_publisher
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()

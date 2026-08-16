cmake_minimum_required(VERSION 3.8)
project(agent_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(agent_state_publisher src/agent_state_publisher.cpp)
ament_target_dependencies(
  agent_state_publisher
  geometry_msgs
  rclcpp
  swarm_interfaces
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    agent_state_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

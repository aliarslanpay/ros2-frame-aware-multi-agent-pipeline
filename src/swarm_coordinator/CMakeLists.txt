cmake_minimum_required(VERSION 3.13)
project(swarm_coordinator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(
  SWARM_COORDINATOR_ENABLE_TSAN
  "Build swarm_coordinator targets with ThreadSanitizer"
  OFF
)
if(SWARM_COORDINATOR_ENABLE_TSAN)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
    add_link_options(-fsanitize=thread)
  else()
    message(FATAL_ERROR "ThreadSanitizer requires GCC or Clang")
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(swarm_core REQUIRED)
find_package(swarm_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(agent_state_cache STATIC src/agent_state_cache.cpp)
target_include_directories(agent_state_cache
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(agent_state_cache builtin_interfaces swarm_interfaces)
target_link_libraries(agent_state_cache swarm_core::swarm_core)

add_library(state_frame_normalizer STATIC src/state_frame_normalizer.cpp)
target_include_directories(state_frame_normalizer
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  state_frame_normalizer
  builtin_interfaces
  geometry_msgs
  swarm_interfaces
  tf2
  tf2_geometry_msgs
)

add_library(
  frame_aware_state_processor STATIC
  src/frame_aware_state_processor.cpp
)
target_include_directories(frame_aware_state_processor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
  frame_aware_state_processor
  agent_state_cache
  state_frame_normalizer
)
ament_target_dependencies(frame_aware_state_processor swarm_interfaces)

add_executable(swarm_coordinator_node src/swarm_coordinator_node.cpp)
target_link_libraries(swarm_coordinator_node frame_aware_state_processor)
ament_target_dependencies(
  swarm_coordinator_node
  builtin_interfaces
  rclcpp
  swarm_interfaces
  tf2
  tf2_ros
)

install(
  TARGETS swarm_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Threads REQUIRED)

  ament_add_gtest(test_agent_state_cache test/test_agent_state_cache.cpp)
  target_link_libraries(test_agent_state_cache agent_state_cache)
  ament_target_dependencies(test_agent_state_cache swarm_interfaces)

  ament_add_gtest(
    test_agent_state_cache_concurrency
    test/test_agent_state_cache_concurrency.cpp
  )
  target_link_libraries(
    test_agent_state_cache_concurrency
    agent_state_cache
    Threads::Threads
  )
  ament_target_dependencies(test_agent_state_cache_concurrency swarm_interfaces)

  ament_add_gtest(
    test_state_frame_normalizer
    test/test_state_frame_normalizer.cpp
  )
  target_link_libraries(test_state_frame_normalizer state_frame_normalizer)
  ament_target_dependencies(
    test_state_frame_normalizer
    geometry_msgs
    swarm_interfaces
    tf2
    tf2_geometry_msgs
  )

  ament_add_gtest(
    test_frame_aware_state_processor
    test/test_frame_aware_state_processor.cpp
  )
  target_link_libraries(
    test_frame_aware_state_processor
    frame_aware_state_processor
  )
  ament_target_dependencies(
    test_frame_aware_state_processor
    geometry_msgs
    swarm_interfaces
    tf2
    tf2_geometry_msgs
  )
endif()

ament_package()

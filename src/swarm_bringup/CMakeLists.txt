cmake_minimum_required(VERSION 3.8)
project(swarm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_multi_agent_pipeline.py
    TARGET test_multi_agent_pipeline
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    TIMEOUT 35
  )
endif()

ament_package()

cmake_minimum_required(VERSION 3.16)
project(wsl_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav2_config test/test_nav2_config.py
    ENV WSL_NAVIGATION_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60)

  # Drives the full system to a goal: needs an OpenGL renderer and takes
  # minutes, so it is opt-in like the wsl_sim smoke test. It launches wsl_sim,
  # which depends on this package, so wsl_sim cannot be declared as a test
  # dependency (colcon rejects the cycle): source install/setup.bash first.
  option(WSL_SIM_TESTS "Run the headless end-to-end simulation tests" OFF)
  if(WSL_SIM_TESTS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_navigate_to_goal.launch.py TIMEOUT 900)
  endif()
endif()

ament_package()

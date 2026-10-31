cmake_minimum_required(VERSION 3.16)
project(wsl_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models rviz worlds DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sim_assets test/test_sim_assets.py
    ENV WSL_SIM_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    TIMEOUT 60)

  # Full-system test: needs an OpenGL renderer and takes minutes, so it is opt-in.
  option(WSL_SIM_TESTS "Run the headless end-to-end simulation smoke test" OFF)
  if(WSL_SIM_TESTS)
    find_package(launch_testing_ament_cmake REQUIRED)
    add_launch_test(test/test_system_smoke.launch.py TIMEOUT 600)
  endif()
endif()

ament_package()

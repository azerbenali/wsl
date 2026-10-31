cmake_minimum_required(VERSION 3.16)
# C is enabled for dependencies, not for our code: on Ubuntu 24.04, PCL pulls in
# VTK 9.1, whose exported targets link MPI::MPI_C, and FindMPI only defines that
# target when the C language is enabled.
project(wsl_perception LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(WSL_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(WSL_BUILD_FUZZERS "Build the libFuzzer target (requires clang)" OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(WSL_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree sample_consensus search segmentation)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# ROS-independent algorithms: depends on PCL and the sensor_msgs message type only.
add_library(wsl_perception_core STATIC
  src/cloud_pipeline.cpp
  src/point_cloud_conversion.cpp)
set_target_properties(wsl_perception_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(wsl_perception_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
# PCL's CMake config exports plain variables rather than targets.
target_include_directories(wsl_perception_core SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(wsl_perception_core PUBLIC ${PCL_DEFINITIONS})
target_link_libraries(wsl_perception_core PUBLIC
  ${PCL_LIBRARIES} Eigen3::Eigen ${sensor_msgs_TARGETS})

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node PRIVATE
  wsl_perception_core
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  diagnostic_updater::diagnostic_updater
  ${vision_msgs_TARGETS}
  ${visualization_msgs_TARGETS})

# pcl_conversions is header-only and exports variables, not a target.
target_include_directories(perception_node SYSTEM PRIVATE ${pcl_conversions_INCLUDE_DIRS})

add_executable(pipeline_benchmark bench/pipeline_benchmark.cpp)
target_include_directories(pipeline_benchmark PRIVATE test)
target_link_libraries(pipeline_benchmark PRIVATE wsl_perception_core)

install(TARGETS perception_node pipeline_benchmark DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(WSL_BUILD_FUZZERS)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "WSL_BUILD_FUZZERS requires clang (-DCMAKE_CXX_COMPILER=clang++)")
  endif()
  add_executable(fuzz_point_cloud_conversion fuzz/fuzz_point_cloud_conversion.cpp)
  target_compile_options(fuzz_point_cloud_conversion PRIVATE -fsanitize=fuzzer,address,undefined)
  target_link_options(fuzz_point_cloud_conversion PRIVATE -fsanitize=fuzzer,address,undefined)
  target_link_libraries(fuzz_point_cloud_conversion PRIVATE wsl_perception_core)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name test_point_cloud_conversion test_cloud_pipeline test_pipeline_properties)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 120)
    target_include_directories(${test_name} PRIVATE test)
    target_link_libraries(${test_name} wsl_perception_core)
  endforeach()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_perception_node.launch.py TIMEOUT 120)
endif()

ament_package()

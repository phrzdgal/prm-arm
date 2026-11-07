cmake_minimum_required(VERSION 3.22)
project(precision_robotic_manipulation VERSION 0.1.0 LANGUAGES CXX)

# C++20 matches the minimum language standard for ROS 2 Lyrical,
# so nothing here will need to change when we move into ROS 2.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Treat warnings seriously from day one. Safety-critical teams do.
add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# --- Kinematics library ------------------------------------------------------
# Kept separate from the CLI so the same code can later be linked into
# ROS 2 nodes and compared against MoveIt's results.
add_library(prm_kinematics src/two_link_arm.cpp)
target_include_directories(prm_kinematics PUBLIC include)

# --- Command-line demo -------------------------------------------------------
add_executable(fk_cli app/main.cpp)
target_link_libraries(fk_cli PRIVATE prm_kinematics)

# --- Tests -------------------------------------------------------------------
option(PRM_BUILD_TESTS "Build unit tests" ON)
if(PRM_BUILD_TESTS)
  enable_testing()
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)
  endif()

  add_executable(test_two_link_arm tests/test_two_link_arm.cpp)
  target_link_libraries(test_two_link_arm PRIVATE prm_kinematics GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(test_two_link_arm)
endif()

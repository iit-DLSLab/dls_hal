cmake_minimum_required(VERSION 3.16.2)
project(dls_hal)
add_compile_options("-std=c++20")

option(DLS_HAL_BUILD_PYTHON_BINDINGS "Build the nanobind Python bindings" OFF)

add_subdirectory(hal)
add_subdirectory(robot_interface)
add_subdirectory(fastdds_high_level_interface)

if(DLS_HAL_BUILD_PYTHON_BINDINGS)
  add_subdirectory(python)
endif()

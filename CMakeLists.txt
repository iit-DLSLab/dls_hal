cmake_minimum_required(VERSION 3.28.3)
project(dls_hal_interface)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")
set(CMAKE_INSTALL_PREFIX /opt/dls2 CACHE PATH "Install path" FORCE)
# append the path to the cmake modules of dls2 to the CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}"/opt/dls2/cmake" CACHE PATH "Install path" FORCE)

add_compile_options("-std=c++20")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# get default relative layout
include(GNUInstallDirs)
include(hal_install_target)

option(DLS_HAL_BUILD_PYTHON_BINDINGS "Build the nanobind Python bindings" OFF)

add_subdirectory(hal)
add_subdirectory(robot_interface)
add_subdirectory(fastdds_high_level_interface)
add_subdirectory(test/test_hal)

if(DLS_HAL_BUILD_PYTHON_BINDINGS)
  add_subdirectory(python)
endif()

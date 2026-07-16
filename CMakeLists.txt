cmake_minimum_required(VERSION 3.16.2)
project(dls_hal)

add_library(dls_hal
    src/robot_invariants.cpp
)

target_link_libraries(dls_hal PUBLIC yaml-cpp)

target_include_directories(dls_hal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_subdirectory(test)

cmake_minimum_required(VERSION 3.16.2)
project(dls_hal)
add_compile_options("-std=c++20")

add_subdirectory(robot_interface)
add_subdirectory(fastdds_high_level_interface)

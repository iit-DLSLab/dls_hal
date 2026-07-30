cmake_minimum_required(VERSION 3.16.2)
project(robot_interface)

add_library(robot_interface SHARED
    src/robot_invariants.cpp
)

set_target_properties(robot_interface PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(robot_interface PUBLIC yaml-cpp)

target_include_directories(robot_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS robot_interface
    EXPORT robot_interfaceTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

#install config folder
install(DIRECTORY config
    DESTINATION /usr/share
)
# install include folder
install(DIRECTORY include/
    DESTINATION /usr/include
)

add_subdirectory(test)

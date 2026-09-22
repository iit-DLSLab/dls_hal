set(BUILD_SHARED_LIBS ON)

add_library(robot_interface
    src/robot_invariants.cpp
    src/recovery_modes.cpp
)
# create an alias for the library to be used by other packages
add_library(${PROJECT_NAME}::robot_interface ALIAS robot_interface)

target_link_libraries(robot_interface PUBLIC yaml-cpp)

target_include_directories(robot_interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

hal_install_target(robot_interface)

#install config folder
install(DIRECTORY config
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/dls_hal
)

add_subdirectory(test)

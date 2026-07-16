
add_executable(test_robot_interface
    test_robot_interface.cpp
)

target_link_libraries(test_robot_interface PRIVATE dls_hal)
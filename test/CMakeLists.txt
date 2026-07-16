
add_executable(test_robot_interface
    test_robot_interface.cpp
)

target_link_libraries(test_robot_interface PRIVATE dls_hal)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/quadruped_interface_config.yaml
    ${CMAKE_CURRENT_BINARY_DIR}/quadruped_interface_config.yaml
    COPYONLY
)
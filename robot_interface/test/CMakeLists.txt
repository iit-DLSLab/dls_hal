
add_executable(test_robot_interface
    test_robot_interface.cpp
)

target_link_libraries(test_robot_interface PRIVATE 
    robot_interface
)

if(BUILD_TESTING)
    add_executable(test_recovery_procedure test_recovery_procedure.cpp)
    target_link_libraries(test_recovery_procedure PRIVATE robot_interface)
    add_test(NAME recovery_procedure COMMAND test_recovery_procedure)
endif()

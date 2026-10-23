include(GoogleTest)

add_executable(test_robot_invariants test_robot_invariants.cpp)
target_link_libraries(test_robot_invariants PRIVATE robot_interface GTest::gtest_main)

# Register each TEST as its own CTest entry, so `ctest` lists every anomaly.
gtest_discover_tests(test_robot_invariants)

# `make run_robot_invariants_tests` builds and runs the suite with gtest output.
add_custom_target(run_robot_invariants_tests
    COMMAND test_robot_invariants --gtest_color=yes
    DEPENDS test_robot_invariants
    USES_TERMINAL
)

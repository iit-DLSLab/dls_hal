set(PLUGIN_NAME hal_anomaly_tester)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(
    COMMAND
        ${Python3_EXECUTABLE} -c "import sysconfig; schemes = sysconfig.get_scheme_names(); scheme = 'deb_system' if 'deb_system' in schemes else sysconfig.get_default_scheme(); print(sysconfig.get_path('purelib', scheme=scheme))"
    OUTPUT_VARIABLE
        HAL_ANOMALY_TESTER_DLS_PYTHON_ROOT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(HAL_ANOMALY_TESTER_SHARED_PYTHON /usr/lib/dls2/python/bin/python3)
set(HAL_ANOMALY_TESTER_RUNTIME_ROOT /usr/share/${PLUGIN_NAME})

configure_file(
    cmake/${PLUGIN_NAME}.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}
    @ONLY
)

install(
    FILES       plugin/config/scheduler.yaml
    DESTINATION /usr/include/dls2/schedulers/
    RENAME      ${PLUGIN_NAME}.yaml
)

install(
    PROGRAMS    ${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME} scripts/hal_test_config
    DESTINATION /usr/bin
)

install(
    FILES       python/hal_anomaly_tester.py python/anomaly_tests.py python/simple_yaml.py
    DESTINATION ${HAL_ANOMALY_TESTER_RUNTIME_ROOT}/python
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(
    FILES       config/${PLUGIN_NAME}.json config/${PLUGIN_NAME}_sim.json
    DESTINATION ${HAL_ANOMALY_TESTER_RUNTIME_ROOT}/config
)

# Runner logic against a fake HAL (no DDS needed).
if(DLS_HAL_BUILD_TESTS)
    add_test(NAME hal_anomaly_tester_runner
        COMMAND ${Python3_EXECUTABLE} -B -m unittest test_anomaly_tests -v
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python
    )
endif()

cmake_minimum_required(VERSION 3.16.2)
project(dls_hal)


set(DEFAULT_ROBOT_INTERFACE_PATH "/usr/share/config/dls_hal")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dls_hal/config.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/dls_hal/config.hpp
    @ONLY
)
add_library(dls_hal SHARED
    src/robot_invariants.cpp
)

set_target_properties(dls_hal PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(dls_hal PUBLIC yaml-cpp)

target_include_directories(dls_hal PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(TARGETS dls_hal
    EXPORT dls_halTargets
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

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/include/dls_hal/config.hpp
    DESTINATION
        /usr/include/dls_hal
)

add_subdirectory(test)

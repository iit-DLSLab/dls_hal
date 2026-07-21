cmake_minimum_required(VERSION 3.16.2)
project(fastdds_high_level_interface)

add_library(${PROJECT_NAME} SHARED
    src/fastdds_high_level_interface.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME}
INTERFACE
    transport_interfaces
PUBLIC
    robot_interface
    fastdds_transport
    dls_topics
    dls_signal
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_directories(${PROJECT_NAME} PUBLIC
    /usr/lib/dls2
    /usr/local/lib
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# install include folder
install(DIRECTORY include/
    DESTINATION /usr/include
)
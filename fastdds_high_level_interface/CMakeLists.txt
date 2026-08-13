add_library(fastdds_high_level_interface SHARED
    src/fastdds_high_level_interface.cpp
)
# add an alias for the library to be used by other packages
add_library(${PROJECT_NAME}::fastdds_high_level_interface ALIAS fastdds_high_level_interface)

if (NOT TARGET transport_interfaces::transport_interface)
    find_package(transport_interface REQUIRED)
endif()
if (NOT TARGET transport_interfaces::fastdds_transport)
    find_package(fastdds_transport REQUIRED)
endif()

target_link_libraries(fastdds_high_level_interface
PUBLIC
    transport_interfaces::transport_interface
PUBLIC
    ${PROJECT_NAME}::robot_interface
    transport_interfaces::fastdds_transport
)

target_include_directories(fastdds_high_level_interface
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

hal_install_target(fastdds_high_level_interface)

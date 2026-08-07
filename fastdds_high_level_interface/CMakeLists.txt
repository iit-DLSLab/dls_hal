add_library(fastdds_high_level_interface SHARED
    src/fastdds_high_level_interface.cpp
)
# append the path to the cmake modules of dls2 to the CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}"/opt/dls2/cmake" CACHE PATH "Install path" FORCE)

if (NOT TARGET transport_interfaces)
    find_package(transport_interfaces REQUIRED)
endif()
if (NOT TARGET robot_interface)
    find_package(robot_interface REQUIRED)
endif()
if (NOT TARGET fastdds_transport)
    find_package(fastdds_transport REQUIRED)
endif()

target_link_libraries(fastdds_high_level_interface
PUBLIC
    transport_interfaces
PUBLIC
    robot_interface
    fastdds_transport
)

target_include_directories(fastdds_high_level_interface
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

hal_install_target(fastdds_high_level_interface)
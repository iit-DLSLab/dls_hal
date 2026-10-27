add_library(fastdds_high_level_interface SHARED
    src/fastdds_event_checker.cpp
)
# add an alias for the library to be used by other packages
add_library(${PROJECT_NAME}::fastdds_high_level_interface ALIAS fastdds_high_level_interface)

if (NOT TARGET transport_interfaces::transport_interface)
    find_package(transport_interface REQUIRED)
endif()
if (NOT TARGET transport_interfaces::fastdds_transport)
    find_package(fastdds_transport REQUIRED)
endif()

find_package(Threads REQUIRED)

target_link_libraries(fastdds_high_level_interface
PUBLIC
    transport_interfaces::transport_interface
PUBLIC
    ${PROJECT_NAME}::robot_interface
    transport_interfaces::fastdds_transport
    # dls_logging is installed by dls2 but is not part of a CMake export set.
    # Propagate it only within the build tree, not to installed consumers.
    $<BUILD_INTERFACE:dls_logging>
    # dls::CommandManager in the public header; like dls_logging, not part of a CMake export set.
    $<BUILD_INTERFACE:dls_command>
PRIVATE
    Threads::Threads
)

# std::jthread/std::stop_token in the public header
target_compile_features(fastdds_high_level_interface PUBLIC cxx_std_20)

target_include_directories(fastdds_high_level_interface
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
)

hal_install_target(fastdds_high_level_interface)

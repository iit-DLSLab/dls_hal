include(CMakeFindDependencyMacro)

find_dependency(transport_interface CONFIG)
find_dependency(fastdds_transport CONFIG)
find_dependency(robot_interface CONFIG)

include("${CMAKE_CURRENT_LIST_DIR}/fastdds_high_level_interfaceTargets.cmake")

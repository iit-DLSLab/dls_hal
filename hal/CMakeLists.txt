set(DEFAULT_DLS_HAL_INTERFACE_PATH "/usr/include/dls2/hardwares")

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/dls_hal/config_interface_path.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/dls_hal/config_interface_path.hpp
    @ONLY
)

add_library(dls_hal INTERFACE)

target_include_directories(dls_hal INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

hal_install_target(dls_hal)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/include/dls_hal/config_interface_path.hpp
    DESTINATION
        hardwares/${CMAKE_INSTALL_INCLUDEDIR}
)
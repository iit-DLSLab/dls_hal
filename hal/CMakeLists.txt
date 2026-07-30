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
  $<INSTALL_INTERFACE:include>
)

install(TARGETS dls_hal
  EXPORT dls_hal_halTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION /usr/include
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/include/dls_hal/config_interface_path.hpp
    DESTINATION
        /usr/include/dls2/hardwares
)
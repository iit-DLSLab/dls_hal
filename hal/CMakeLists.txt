add_library(dls_hal INTERFACE)

add_library(${PROJECT_NAME}::dls_hal ALIAS dls_hal)

target_include_directories(dls_hal INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

hal_install_target(dls_hal)
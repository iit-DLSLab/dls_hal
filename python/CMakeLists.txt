find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(dls_hal_python bindings.cpp)
set_target_properties(dls_hal_python PROPERTIES OUTPUT_NAME dls_hal)

target_link_libraries(dls_hal_python PRIVATE
  dls_hal
  robot_interface
)

install(TARGETS dls_hal_python
  LIBRARY DESTINATION lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages
)

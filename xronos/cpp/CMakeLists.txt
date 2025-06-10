find_package(Python 3.10.0...<3.14.0 REQUIRED COMPONENTS Interpreter
                                                         Development.Module)
find_package(pybind11 CONFIG REQUIRED)

python_add_library(
  _cpp_sdk
  MODULE
  cpp_sdk.cc
  WITH_SOABI)
target_link_libraries(
  _cpp_sdk
  PRIVATE xronos::xronos-sdk xronos::xronos-runtime xronos::xronos-telemetry xronos::xronos-telemetry-otel pybind11::headers
          xronos::xronos-graph-exporter xronos::xronos-graph-messages)

target_include_directories(_cpp_sdk PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
set_target_properties(_cpp_sdk PROPERTIES
  CXX_CLANG_TIDY "${XRONOS_CLANG_TIDY_COMMAND}"
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

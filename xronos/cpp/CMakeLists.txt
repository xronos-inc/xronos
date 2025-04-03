find_package(Python 3.10.0...<3.14.0 REQUIRED COMPONENTS Interpreter
                                                         Development.Module)
find_package(pybind11 CONFIG REQUIRED)

python_add_library(
  _runtime
  MODULE
  action.cc
  environment.cc
  metric.cc
  module.cc
  port.cc
  reaction.cc
  reactor.cc
  reactor_element.cc
  WITH_SOABI)
target_link_libraries(
  _runtime
  PRIVATE xronos::xronos-runtime xronos::xronos-telemetry xronos::xronos-telemetry-otel pybind11::headers
          xronos::xronos-graph-exporter xronos::xronos-graph-messages)

target_include_directories(_runtime PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
set_target_properties(_runtime PROPERTIES CXX_CLANG_TIDY
  "${XRONOS_CLANG_TIDY_COMMAND}")

if(MSVC)
  target_compile_options(_runtime PRIVATE /W4)
else()
  target_compile_options(_runtime PRIVATE -Wall -Wextra -pedantic)
endif()

add_library(
  xronos-telemetry-otel
  lib/common.cc lib/otel_metric_data_logger.cc lib/otel_runtime_data_logger.cc
  lib/otel_telemetry_backend.cc)
target_link_libraries(
  xronos-telemetry-otel
  PUBLIC xronos::xronos-telemetry xronos::xronos-runtime
  PRIVATE
    opentelemetry-cpp::api
    opentelemetry-cpp::proto
    opentelemetry-cpp::proto_grpc
    opentelemetry-cpp::otlp_grpc_exporter
)
target_include_directories(
  xronos-telemetry-otel
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  PRIVATE "${CMAKE_CURRENT_LIST_DIR}/lib/include")

if(XRONOS_LIB_RUN_CLANG_TIDY)
  set_target_properties(xronos-telemetry-otel
    PROPERTIES CXX_CLANG_TIDY "${XRONOS_CLANG_TIDY_COMMAND}")
endif()
set_target_properties(xronos-telemetry-otel PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_INSTALL_HEADERS)
  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()

if(XRONOS_LIB_BUILD_TESTS)
  add_executable(xronos-telemetry-otel-tests test/test_attributes.cc)
  target_link_libraries(
    xronos-telemetry-otel-tests
    PRIVATE xronos-telemetry-otel xronos::xronos-runtime Catch2::Catch2WithMain opentelemetry-cpp::api
    opentelemetry-cpp::sdk)
  target_include_directories(xronos-telemetry-otel-tests
    PRIVATE "${CMAKE_CURRENT_LIST_DIR}/lib/include")
  catch_discover_tests(xronos-telemetry-otel-tests)
endif()

add_library(xronos::xronos-telemetry-otel ALIAS xronos-telemetry-otel)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")
file(GLOB_RECURSE PRIVATE_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/lib/include/*.hh")

add_library(xronos-telemetry-otel ${SOURCES})
target_link_libraries(
  xronos-telemetry-otel
  PUBLIC xronos::xronos-telemetry xronos::xronos-runtime-interfaces
  PRIVATE xronos::xronos-util opentelemetry-cpp::api opentelemetry-cpp::proto opentelemetry-cpp::proto_grpc opentelemetry-cpp::otlp_grpc_exporter
)
target_compile_options(xronos-telemetry-otel PRIVATE -Wall -Wextra -pedantic -Werror)
target_sources(
  xronos-telemetry-otel
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}

  PRIVATE FILE_SET private_headers
  TYPE HEADERS
  BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
  FILES ${PRIVATE_HEADERS}
)

set_target_properties(xronos-telemetry-otel PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)
set_target_properties(xronos-telemetry-otel PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

add_library(xronos::xronos-telemetry-otel ALIAS xronos-telemetry-otel)

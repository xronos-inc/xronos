file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-backend ${SOURCES})
target_sources(
  xronos-backend
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(
  xronos-backend
  PUBLIC xronos-abi xronos-core xronos-runtime-interfaces xronos-source-location xronos-telemetry
  PRIVATE xronos-util xronos-validator
)
if(XRONOS_LIB_WITH_OTEL)
  target_link_libraries(xronos-backend PRIVATE xronos-telemetry-otel)
  target_compile_definitions(xronos-backend PRIVATE XRONOS_BACKEND_ENABLE_TELEMETRY)
endif()
if(XRONOS_LIB_WITH_GRAPH_EXPORTER)
  target_link_libraries(xronos-backend PRIVATE xronos-graph-exporter)
  target_compile_definitions(xronos-backend PRIVATE XRONOS_BACKEND_ENABLE_DIAGRAMS)
endif()
target_compile_options(xronos-backend PRIVATE -Wall $<$<BOOL:${XRONOS_STRICT_WARNINGS}>:-Wextra -pedantic -Werror>)
set_target_properties(xronos-backend PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

set_target_properties(xronos-backend PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_BUILD_TESTS)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
  add_executable(xronos-backend-tests ${TEST_SOURCES})
  target_link_libraries(
    xronos-backend-tests
    PRIVATE xronos-backend xronos-runtime-default xronos-value Catch2::Catch2WithMain
  )
  catch_discover_tests(xronos-backend-tests)
endif()

add_library(xronos::xronos-backend ALIAS xronos-backend)

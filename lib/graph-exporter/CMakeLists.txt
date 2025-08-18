file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-graph-exporter ${SOURCES})
target_link_libraries(xronos-graph-exporter
                      PUBLIC xronos-runtime
                             xronos-graph-messages
                             xronos-telemetry)
target_sources(
  xronos-graph-exporter
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_compile_options(xronos-graph-exporter PRIVATE -Wall -Wextra -pedantic -Werror -Wno-error=unused-parameter)
set_target_properties(xronos-graph-exporter PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)
set_target_properties(xronos-graph-exporter PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)


file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
set(GRAPH_EXPORTER_TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)  # make the variable available in parent CMakeLists.txt
if(XRONOS_LIB_BUILD_TESTS)
  add_executable(xronos-graph-exporter-tests ${TEST_SOURCES})
  target_link_libraries(
    xronos-graph-exporter-tests
    PRIVATE xronos-runtime xronos-graph-exporter xronos-graph-messages xronos-telemetry
            Catch2::Catch2WithMain)
  catch_discover_tests(xronos-graph-exporter-tests)
endif()

add_library(xronos::xronos-graph-exporter ALIAS xronos-graph-exporter)

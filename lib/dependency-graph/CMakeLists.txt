file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-dependency-graph ${SOURCES})
target_sources(
  xronos-dependency-graph
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(xronos-dependency-graph
                      PUBLIC
                        xronos-core
                        nonstd::expected-lite
                      PRIVATE
                        xronos-util)
target_compile_options(xronos-dependency-graph PRIVATE -Wall $<$<BOOL:${XRONOS_STRICT_WARNINGS}>:-Wextra -pedantic -Werror>)
set_target_properties(xronos-dependency-graph PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)
set_target_properties(xronos-dependency-graph PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_BUILD_TESTS)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
  add_executable(xronos-dependency-graph-tests ${TEST_SOURCES})
  target_link_libraries(xronos-dependency-graph-tests PRIVATE xronos-dependency-graph Catch2::Catch2WithMain)
  catch_discover_tests(xronos-dependency-graph-tests)
endif()

add_library(xronos::xronos-dependency-graph ALIAS xronos-dependency-graph)

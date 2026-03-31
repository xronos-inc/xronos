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
target_compile_options(xronos-dependency-graph PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-dependency-graph PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)
set_target_properties(xronos-dependency-graph PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-dependency-graph ALIAS xronos-dependency-graph)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
configure_file(cmake/config.hh.in include/xronos/util/gen/config.hh @ONLY)

file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh" "${CMAKE_CURRENT_BINARY_DIR}/include/*.hh")

add_library(xronos-util ${SOURCES})
target_sources(
  xronos-util
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_compile_options(xronos-util PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-util PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)
set_target_properties(xronos-util PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-util ALIAS xronos-util)

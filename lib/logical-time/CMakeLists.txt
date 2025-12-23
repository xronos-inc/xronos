file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-logical-time ${SOURCES})
target_sources(
  xronos-logical-time
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

target_compile_options(xronos-logical-time PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-logical-time PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)
set_target_properties(xronos-logical-time PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

target_link_libraries(xronos-logical-time PUBLIC xronos::xronos-core PRIVATE xronos::xronos-util)

add_library(xronos::xronos-logical-time ALIAS xronos-logical-time)

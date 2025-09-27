file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh" "${CMAKE_CURRENT_BINARY_DIR}/include/*.hh")

add_library(xronos-runtime-default ${SOURCES})
target_sources(
  xronos-runtime-default
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

add_subdirectory(impl)

target_link_libraries(xronos-runtime-default
  PUBLIC
     xronos::xronos-core
     xronos::xronos-runtime-interfaces
  PRIVATE
     xronos::xronos-runtime-default-impl
     xronos::xronos-util)
target_compile_options(xronos-runtime-default PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-runtime-default PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

set_target_properties(xronos-runtime-default PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-runtime-default ALIAS xronos-runtime-default)

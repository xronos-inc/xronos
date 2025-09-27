file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh" "${CMAKE_CURRENT_BINARY_DIR}/include/*.hh")

add_library(xronos-runtime-default-impl ${SOURCES})
target_sources(
  xronos-runtime-default-impl
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

find_package(Threads)
target_link_libraries(
  xronos-runtime-default-impl
  PUBLIC
    ${CMAKE_THREAD_LIBS_INIT}
    xronos::xronos-core
    xronos::xronos-runtime-interfaces
    xronos::xronos-util
)
target_compile_options(xronos-runtime-default-impl PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-runtime-default-impl PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

configure_file(cmake/config.hh.in include/xronos/runtime/default/impl/gen/config.hh @ONLY)

set_target_properties(xronos-runtime-default-impl PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-runtime-default-impl ALIAS xronos-runtime-default-impl)

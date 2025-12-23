file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")
file(GLOB_RECURSE PRIVATE_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.hh")

add_library(xronos-runtime-default ${SOURCES})
target_sources(
  xronos-runtime-default
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

add_library(xronos-runtime-default-private-headers INTERFACE)
target_sources(
  xronos-runtime-default-private-headers
  INTERFACE FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
  FILES ${PRIVATE_HEADERS}
)
target_include_directories(xronos-runtime-default-private-headers INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/include)
target_link_libraries(xronos-runtime-default-private-headers
  INTERFACE
  xronos::xronos-core
  xronos::xronos-logical-time
  xronos::xronos-util
  xronos::xronos-runtime-interfaces
)
set_target_properties(xronos-runtime-default-private-headers PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)


target_link_libraries(xronos-runtime-default
  PUBLIC
  xronos::xronos-core
  xronos::xronos-runtime-interfaces
  PRIVATE
  xronos::xronos-logical-time
  xronos::xronos-util)
target_include_directories(xronos-runtime-default PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>)
target_compile_options(xronos-runtime-default PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-runtime-default PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

set_target_properties(xronos-runtime-default PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-runtime-default ALIAS xronos-runtime-default)

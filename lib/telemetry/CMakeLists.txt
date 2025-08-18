file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-telemetry ${SOURCES})
target_sources(
  xronos-telemetry
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(xronos-telemetry PUBLIC xronos-runtime)
target_compile_options(xronos-telemetry PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-telemetry PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

set_target_properties(xronos-telemetry PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-telemetry ALIAS xronos-telemetry)

file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-runtime-interfaces INTERFACE)
target_sources(
  xronos-runtime-interfaces
  INTERFACE FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(xronos-runtime-interfaces INTERFACE xronos-core)

set_target_properties(xronos-runtime-interfaces PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

add_library(xronos::xronos-runtime-interfaces ALIAS xronos-runtime-interfaces)

file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-source-location INTERFACE)
target_sources(
  xronos-source-location
  INTERFACE FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(xronos-source-location INTERFACE xronos-util)

set_target_properties(xronos-source-location PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

add_library(xronos::xronos-source-location ALIAS xronos-source-location)

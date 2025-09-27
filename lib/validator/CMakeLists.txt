file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-validator ${SOURCES})
target_link_libraries(xronos-validator
                      PUBLIC
                        xronos-core
                        nonstd::expected-lite
                      PRIVATE
                        fmt::fmt-header-only
)
target_sources(
  xronos-validator
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_compile_options(xronos-validator PRIVATE -Wall -Wextra -pedantic -Werror -Wno-error=unused-parameter)
set_target_properties(xronos-validator PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)
set_target_properties(xronos-validator PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

add_library(xronos::xronos-validator ALIAS xronos-validator)

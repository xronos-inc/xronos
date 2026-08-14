file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-core ${SOURCES})
target_sources(
  xronos-core
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(xronos-core PUBLIC xronos-abi xronos-util nonstd::expected-lite PRIVATE fmt::fmt-header-only)

target_compile_options(xronos-core PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-core PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)
set_target_properties(xronos-core PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_BUILD_TESTS)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
  add_executable(xronos-core-tests ${TEST_SOURCES})
  target_link_libraries(xronos-core-tests PRIVATE xronos-core Catch2::Catch2WithMain)
  catch_discover_tests(xronos-core-tests)
endif()

add_library(xronos::xronos-core ALIAS xronos-core)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-physical-time ${SOURCES})
target_sources(
  xronos-physical-time
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

target_compile_options(xronos-physical-time PRIVATE -Wall $<$<BOOL:${XRONOS_STRICT_WARNINGS}>:-Wextra -pedantic -Werror>)
set_target_properties(xronos-physical-time PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)
set_target_properties(xronos-physical-time PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

target_link_libraries(xronos-physical-time PUBLIC xronos::xronos-core)

if(XRONOS_LIB_BUILD_TESTS)
  include(../../third-party/googletest/googletest.cmake)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
  add_executable(xronos-physical-time-tests ${TEST_SOURCES})
  target_link_libraries(xronos-physical-time-tests PRIVATE xronos-physical-time GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(xronos-physical-time-tests)
endif()

add_library(xronos::xronos-physical-time ALIAS xronos-physical-time)

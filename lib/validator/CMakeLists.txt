file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-validator ${SOURCES})
target_link_libraries(xronos-validator
                      PUBLIC
                        xronos-core
                        nonstd::expected-lite
                      PRIVATE
                        fmt::fmt-header-only
                        xronos-dependency-graph
)
target_sources(
  xronos-validator
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_compile_options(xronos-validator PRIVATE -Wall $<$<BOOL:${XRONOS_STRICT_WARNINGS}>:-Wextra -pedantic -Werror>)
set_target_properties(xronos-validator PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)
set_target_properties(xronos-validator PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

if(XRONOS_LIB_BUILD_TESTS)
  include(../../third-party/googletest/googletest.cmake)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
  add_executable(xronos-validator-tests ${TEST_SOURCES})
  target_link_libraries(xronos-validator-tests PRIVATE xronos-validator GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(xronos-validator-tests)
endif()

add_library(xronos::xronos-validator ALIAS xronos-validator)

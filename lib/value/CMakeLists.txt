file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-value INTERFACE)
target_sources(
  xronos-value
  INTERFACE FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)
target_link_libraries(xronos-value INTERFACE xronos-abi)

set_target_properties(xronos-value PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
set(VALUE_TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)  # make the variable available in parent CMakeLists.txt
if(XRONOS_LIB_BUILD_TESTS)
  add_executable(xronos-value-tests ${TEST_SOURCES})
  target_compile_options(xronos-value-tests PRIVATE -Wall -Wextra -pedantic -Werror)
  target_link_libraries(
    xronos-value-tests
    PRIVATE xronos-value Catch2::Catch2WithMain
  )
  catch_discover_tests(xronos-value-tests)
endif()

add_library(xronos::xronos-value ALIAS xronos-value)

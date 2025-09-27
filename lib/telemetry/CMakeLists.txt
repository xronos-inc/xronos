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
target_link_libraries(xronos-telemetry PUBLIC xronos-core xronos-runtime-interfaces)
target_compile_options(xronos-telemetry PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-telemetry PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

set_target_properties(xronos-telemetry PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.cc")
set(TELEMETRY_TEST_SOURCES ${TEST_SOURCES} PARENT_SCOPE)  # make the variable available in parent CMakeLists.txt
if(XRONOS_LIB_BUILD_TESTS)
  add_executable(xronos-telemetry-tests ${TEST_SOURCES})
  target_link_libraries(
    xronos-telemetry-tests
    PRIVATE xronos-telemetry Catch2::Catch2WithMain
  )
  catch_discover_tests(xronos-telemetry-tests)
endif()

add_library(xronos::xronos-telemetry ALIAS xronos-telemetry)

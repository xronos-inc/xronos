file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.cc")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh" "${CMAKE_CURRENT_BINARY_DIR}/include/*.hh")

add_library(xronos-runtime ${SOURCES})
target_sources(
  xronos-runtime
  PUBLIC FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

find_package(Threads)
target_link_libraries(xronos-runtime PUBLIC ${CMAKE_THREAD_LIBS_INIT})
target_compile_options(xronos-runtime PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-runtime PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

find_package(Backtrace)
set(RUNTIME_USE_BACKTRACE ${Backtrace_FOUND})
if(${Backtrace_FOUND})
  target_include_directories(xronos-runtime PRIVATE ${Backtrace_INCLUDE_DIRS})
  target_link_libraries(xronos-runtime PRIVATE ${Backtrace_LIBRARY})
endif()

configure_file(cmake/config.hh.in include/xronos/runtime/gen/config.hh @ONLY)

set_target_properties(xronos-runtime PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-runtime ALIAS xronos-runtime)

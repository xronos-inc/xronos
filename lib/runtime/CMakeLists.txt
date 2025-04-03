add_library(xronos-runtime
  lib/action.cc
  lib/assert.cc
  lib/environment.cc
  lib/logical_time.cc
  lib/port.cc
  lib/reaction.cc
  lib/reactor.cc
  lib/scheduler.cc
  lib/time.cc
  lib/reactor_element.cc)

find_package(Threads)
target_link_libraries(xronos-runtime PUBLIC ${CMAKE_THREAD_LIBS_INIT})

find_package(Backtrace)
set(RUNTIME_USE_BACKTRACE ${Backtrace_FOUND})
if(${Backtrace_FOUND})
  target_include_directories(xronos-runtime PRIVATE ${Backtrace_INCLUDE_DIRS})
  target_link_libraries(xronos-runtime PRIVATE ${Backtrace_LIBRARY})
endif()

configure_file(cmake/config.hh.in include/xronos/runtime/gen/config.hh @ONLY)

target_include_directories(
  xronos-runtime
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
         "$<INSTALL_INTERFACE:include>")


if(XRONOS_LIB_RUN_CLANG_TIDY)
  set_target_properties(xronos-runtime PROPERTIES CXX_CLANG_TIDY
                                       "${XRONOS_CLANG_TIDY_COMMAND}")
endif()
set_target_properties(xronos-runtime PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_LIB_INSTALL_HEADERS)
  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
  install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()

add_library(xronos::xronos-runtime ALIAS xronos-runtime)

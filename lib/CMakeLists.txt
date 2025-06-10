cmake_minimum_required(VERSION 3.28)

project(xronos-lib LANGUAGES CXX VERSION "0.4.0")
set(PROJECT_VERSION "0.4.0")

# require C++ 20
set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Colorize compilation output
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Generate compilation database for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS "Build using shared libraries" Off)
option(CLANG_TIDY_ERROR "clang-tidy warnings are errors" Off)
option(RUNTIME_VALIDATE "Enable runtime validation" ON)
set(CLANG_TIDY_COMMAND "clang-tidy" CACHE STRING "The clang-tidy command to use")
set(RUNTIME_LOG_LEVEL 3 CACHE STRING "The log level used by the runtime")

set(INSTALL_DEFAULT ON)
set(BUILD_TESTS_DEFAULT ON)
set(RUN_CLANG_TIDY_DEFAULT ON)
if(NOT "${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
  # Disable some features when building as a subproject
  set(INSTALL_DEFAULT OFF)
  set(BUILD_TESTS_DEFAULT OFF)
  set(RUN_CLANG_TIDY_DEFAULT OFF)
endif()

option(XRONOS_LIB_INSTALL "Generate installation target" ${INSTALL_DEFAULT})
option(XRONOS_LIB_INSTALL_HEADERS "Generate installation target for header files" ${INSTALL_DEFAULT})
option(XRONOS_LIB_BUILD_TESTS "Build the tests" ${BUILD_TESTS_DEFAULT})
option(XRONOS_LIB_RUN_CLANG_TIDY "Enable building with clang-tidy" ${RUN_CLANG_TIDY_DEFAULT})

if(XRONOS_LIB_RUN_CLANG_TIDY)
  find_program(CLANG_TIDY ${CLANG_TIDY_COMMAND} REQUIRED)
  if(CLANG_TIDY_ERROR)
    set(CLANG_TIDY_BASE_COMMAND ${CLANG_TIDY_COMMAND}; -warnings-as-errors=*;)
  else()
    set(CLANG_TIDY_BASE_COMMAND ${CLANG_TIDY_COMMAND};)
  endif()
  set(XRONOS_CLANG_TIDY_COMMAND
      "${CLANG_TIDY_BASE_COMMAND};-header-filter=xronos/\(.*\)\\.hh")
endif()

include(GNUInstallDirs)

if(XRONOS_LIB_BUILD_TESTS)
  include(../third-party/catch2/catch2.cmake)
  include(CTest)
  enable_testing()
  include(Catch)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(DEFINED XRONOS_RUNTIME_SOURCE_DIRECTORY)
  include(FetchContent)
  FetchContent_Declare(
    runtime
    SOURCE_DIR "${XRONOS_RUNTIME_SOURCE_DIRECTORY}"
  )
  FetchContent_MakeAvailable(runtime)
else()
  add_subdirectory(runtime)
endif()

set(OPENTELEMETRY_INSTALL ${XRONOS_LIB_INSTALL})
include(../third-party/absl/absl.cmake)
include(../third-party/protobuf/protobuf.cmake)
include(../third-party/grpc/grpc.cmake)
include(../third-party/opentelemetry/opentelemetry.cmake)

find_package(Protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

add_subdirectory(graph-exporter)
add_subdirectory(graph-messages)
add_subdirectory(telemetry)
add_subdirectory(telemetry-otel)

if(XRONOS_LIB_INSTALL)
  install(
    TARGETS
      xronos-runtime
      xronos-telemetry
      xronos-telemetry-otel
      xronos-graph-exporter
      xronos-graph-messages
    EXPORT xronos-libTargets
    ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
    LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
    RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" OPTIONAL)
  export(EXPORT xronos-libTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/xronos-libTargets.cmake"
  )
  install(
    EXPORT xronos-libTargets
    DESTINATION "share/cmake/xronos-lib"
    NAMESPACE xronos::)

  include(CMakePackageConfigHelpers)
  # generate the config file that includes the exports
  configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/xronos-libConfig.cmake"
    INSTALL_DESTINATION "share/cmake/xronos-lib"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/xronos-libConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY ExactVersion
  )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/xronos-libConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/xronos-libConfigVersion.cmake
    DESTINATION "share/cmake/xronos-lib"
  )
endif()

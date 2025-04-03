cmake_minimum_required(VERSION 3.28)

project(xronos-sdk LANGUAGES CXX VERSION "0.1.0")
set(PROJECT_VERSION "0.1.0")

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
set(CLANG_TIDY_COMMAND "clang-tidy" CACHE STRING "The clang-tidy command to use")

set(INSTALL_DEFAULT ON)
set(BUILD_TESTS_DEFAULT ON)
set(RUN_CLANG_TIDY_DEFAULT OFF)
set(BUILD_DOCS_DEFAULT OFF)
set(BUILD_EXAMPLES_DEFAULT ON)
if(NOT "${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
  # Disable some features when building as a subproject
  set(INSTALL_DEFAULT OFF)
  set(BUILD_TESTS_DEFAULT OFF)
  set(RUN_CLANG_TIDY_DEFAULT OFF)
  set(BUILD_EXAMPLES_DEFAULT OFF)
endif()

option(XRONOS_SDK_INSTALL "Generate installation target" ${INSTALL_DEFAULT})
option(XRONOS_SDK_BUILD_TESTS "Build the tests" ${BUILD_TESTS_DEFAULT})
option(XRONOS_SDK_BUILD_EXAMPLES "Build the examples" ${BUILD_EXAMPLES_DEFAULT})
option(XRONOS_SDK_RUN_CLANG_TIDY "Enable building with clang-tidy" ${RUN_CLANG_TIDY_DEFAULT})
option(XRONOS_SDK_BUILD_DOCS "Build API documentation using Doxygen" ${BUILD_DOCS_DEFAULT})

set(XRONOS_LIB_INSTALL ${XRONOS_SDK_INSTALL})
include(FetchContent)
FetchContent_Declare(
  xronos-lib
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/../lib"
  EXCLUDE_FROM_ALL TRUE
  FIND_PACKAGE_ARGS CONFIG
)
FetchContent_MakeAvailable(xronos-lib)

if(XRONOS_SDK_RUN_CLANG_TIDY)
  find_program(CLANG_TIDY ${CLANG_TIDY_COMMAND} REQUIRED)
  if(CLANG_TIDY_ERROR)
    set(CLANG_TIDY_BASE_COMMAND ${CLANG_TIDY_COMMAND}; -warnings-as-errors=*;)
  else()
    set(CLANG_TIDY_BASE_COMMAND ${CLANG_TIDY_COMMAND};)
  endif()
  set(XRONOS_CLANG_TIDY_COMMAND
    "${CLANG_TIDY_BASE_COMMAND};-header-filter=xronos/\(.*\)\\.hh")
endif()

add_compile_options(-Wall -Wextra -pedantic -Werror)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(xronos-sdk
  src/context.cc
  src/element.cc
  src/environment.cc
  src/metric.cc
  src/periodic_timer.cc
  src/reaction.cc
  src/reactor.cc
  src/shutdown.cc
  src/startup.cc
)
target_include_directories(
  xronos-sdk
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(xronos-sdk
  xronos::xronos-runtime
  xronos::xronos-graph-exporter
  xronos::xronos-graph-messages
  xronos::xronos-telemetry
  xronos::xronos-telemetry-otel
)

if(XRONOS_SDK_RUN_CLANG_TIDY)
  set_target_properties(xronos-sdk PROPERTIES CXX_CLANG_TIDY
    "${XRONOS_CLANG_TIDY_COMMAND}")
endif()
set_target_properties(xronos-sdk PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

if(XRONOS_SDK_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(XRONOS_SDK_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

if(XRONOS_SDK_BUILD_DOCS)
  add_subdirectory(docs)
endif()

if(XRONOS_SDK_INSTALL)
  include(GNUInstallDirs)

  install(
    TARGETS xronos-sdk
    EXPORT xronos-sdkTargets
    ARCHIVE  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
    LIBRARY  DESTINATION "${CMAKE_INSTALL_LIBDIR}" OPTIONAL
    RUNTIME  DESTINATION "${CMAKE_INSTALL_BINDIR}" OPTIONAL)
  export(EXPORT xronos-sdkTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/xronos-sdkTargets.cmake"
  )
  install(
    EXPORT xronos-sdkTargets
    DESTINATION "share/cmake/xronos-sdk"
    NAMESPACE xronos::)

  include(CMakePackageConfigHelpers)
  # generate the config file that includes the exports
  configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/xronos-sdkConfig.cmake"
    INSTALL_DESTINATION "share/cmake/xronos-sdk"
    NO_SET_AND_CHECK_MACRO
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
  )
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/xronos-sdkConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY ExactVersion
  )

  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/xronos-sdkConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/xronos-sdkConfigVersion.cmake
    DESTINATION "share/cmake/xronos-sdk"
  )

  install(DIRECTORY include/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
endif()

add_library(xronos::xronos-sdk ALIAS xronos-sdk)

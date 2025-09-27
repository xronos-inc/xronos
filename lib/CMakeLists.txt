cmake_minimum_required(VERSION 3.28)

project(xronos-lib LANGUAGES CXX VERSION "0.6.0")
set(PROJECT_VERSION "0.6.0")

# require C++ 20
set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Colorize compilation output
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Generate compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_SHARED_LIBS "Build using shared libraries" Off)
option(RUNTIME_VALIDATE "Enable runtime validation" ON)
set(XRONOS_LOG_LEVEL 2 CACHE STRING "The xronos internal log level")

set(INSTALL_DEFAULT ON)
set(BUILD_TESTS_DEFAULT ON)
if(NOT "${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
  # Disable some features when building as a subproject
  set(INSTALL_DEFAULT OFF)
  set(BUILD_TESTS_DEFAULT OFF)
endif()

option(XRONOS_LIB_INSTALL "Generate installation target" ${INSTALL_DEFAULT})
option(XRONOS_LIB_INSTALL_HEADERS "Generate installation target for header files" ${INSTALL_DEFAULT})
option(XRONOS_LIB_BUILD_TESTS "Build the tests" ${BUILD_TESTS_DEFAULT})

option(XRONOS_LIB_WITH_GRAPH_EXPORTER "Include the graph exporter in the build" ON)
option(XRONOS_LIB_WITH_OTEL "Include the otel telemetry backend in the build" ON)

include(GNUInstallDirs)

if(XRONOS_LIB_BUILD_TESTS)
  include(../third-party/catch2/catch2.cmake)
  include(CTest)
  enable_testing()
  include(Catch)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(../third-party/expected-lite/expected-lite.cmake)
include(../third-party/fmt/fmt.cmake)

add_subdirectory(core)
add_subdirectory(runtime-default)
add_subdirectory(runtime-interfaces)
add_subdirectory(source-location)
add_subdirectory(telemetry)
add_subdirectory(util)
add_subdirectory(validator)

set(TARGETS
  xronos-core
  xronos-runtime-default
  xronos-runtime-default-impl
  xronos-runtime-interfaces
  xronos-source-location
  xronos-telemetry
  xronos-util
  xronos-validator
)

if(XRONOS_LIB_WITH_GRAPH_EXPORTER)
  include(../third-party/absl/absl.cmake)
  include(../third-party/protobuf/protobuf.cmake)
  include(../third-party/grpc/grpc.cmake)
  add_subdirectory(graph-exporter)
  add_subdirectory(graph-messages)
  set(TARGETS ${TARGETS} xronos-graph-exporter xronos-graph-messages)
endif()

if(XRONOS_LIB_WITH_OTEL)
  include(../third-party/absl/absl.cmake)
  include(../third-party/protobuf/protobuf.cmake)
  include(../third-party/grpc/grpc.cmake)
  include(../third-party/opentelemetry/opentelemetry.cmake)
  add_subdirectory(telemetry-otel)
  set(TARGETS ${TARGETS} xronos-telemetry-otel)
endif()

if(XRONOS_LIB_INSTALL)
  install(
    TARGETS ${TARGETS}
    EXPORT xronos-libTargets
    FILE_SET HEADERS
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

set(LINT_FILES
  $<TARGET_PROPERTY:xronos-core,SOURCES>
  $<TARGET_PROPERTY:xronos-core,HEADER_SET>
  $<TARGET_PROPERTY:xronos-runtime-interfaces,HEADER_SET>
  $<TARGET_PROPERTY:xronos-runtime-default,SOURCES>
  $<TARGET_PROPERTY:xronos-runtime-default,HEADER_SET>
  $<TARGET_PROPERTY:xronos-runtime-default-impl,SOURCES>
  $<TARGET_PROPERTY:xronos-runtime-default-impl,HEADER_SET>
  $<TARGET_PROPERTY:xronos-source-location,HEADER_SET>
  $<TARGET_PROPERTY:xronos-telemetry,SOURCES>
  $<TARGET_PROPERTY:xronos-telemetry,HEADER_SET>
  $<TARGET_PROPERTY:xronos-util,SOURCES>
  $<TARGET_PROPERTY:xronos-util,HEADER_SET>
  $<TARGET_PROPERTY:xronos-validator,SOURCES>
  $<TARGET_PROPERTY:xronos-validator,HEADER_SET>
)

if(XRONOS_LIB_WITH_GRAPH_EXPORTER)
  set(LINT_FILES ${LINT_FILES}
    $<TARGET_PROPERTY:xronos-graph-exporter,SOURCES>
    $<TARGET_PROPERTY:xronos-graph-exporter,HEADER_SET>
  )
endif()
if(XRONOS_LIB_WITH_OTEL)
  set(LINT_FILES ${LINT_FILES}
    $<TARGET_PROPERTY:xronos-telemetry-otel,SOURCES>
    $<TARGET_PROPERTY:xronos-telemetry-otel,HEADER_SET>
    $<TARGET_PROPERTY:xronos-telemetry-otel,HEADER_SET_private_headers>
  )
endif()

set(FORMAT_FILES ${LINT_FILES})
if(XRONOS_LIB_WITH_GRAPH_EXPORTER)
  set(FORMAT_FILES ${FORMAT_FILES} ${GRAPH_EXPORTER_TEST_SOURCES})
endif()
if(XRONOS_LIB_WITH_OTEL)
  set(FORMAT_FILES ${FORMAT_FILES} ${TELEMETRY_TEST_SOURCES})
endif()

add_custom_target(xronos-lib-format
  COMMAND clang-format -i -style=file ${FORMAT_FILES}
  COMMAND_EXPAND_LISTS
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
add_custom_target(xronos-lib-check-format
  COMMAND clang-format --dry-run --Werror -style=file ${FORMAT_FILES}
  COMMAND_EXPAND_LISTS
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

include(ProcessorCount)
ProcessorCount(NCORES)
add_custom_target(xronos-lib-lint COMMAND
  echo ${LINT_FILES} | xargs -n1 -P${NCORES} clang-tidy --quiet -p ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND_EXPAND_LISTS
)

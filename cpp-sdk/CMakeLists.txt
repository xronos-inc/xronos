cmake_minimum_required(VERSION 3.28)

project(xronos-sdk
  LANGUAGES CXX
  VERSION "0.13.0"
  HOMEPAGE_URL "https://docs.xronos.com"
  DESCRIPTION "The Xronos C++ SDK for writing reactor programs."
)
set(PROJECT_VERSION "0.13.0")

# require C++ 20
set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Colorize compilation output
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Generate compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(INSTALL_DEFAULT ON)
set(BUILD_SHARED_DEFAULT ON)
set(BUILD_TESTS_DEFAULT ON)
set(BUILD_DOCS_DEFAULT OFF)
set(BUILD_EXAMPLES_DEFAULT ON)
if(NOT "${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
  # Disable some features when building as a subproject
  set(INSTALL_DEFAULT OFF)
  set(BUILD_SHARED_DEFAULT OFF)
  set(BUILD_TESTS_DEFAULT OFF)
  set(BUILD_EXAMPLES_DEFAULT OFF)
endif()

option(XRONOS_SDK_INSTALL "Generate installation target" ${INSTALL_DEFAULT})
option(XRONOS_SDK_BUILD_SHARED "Build the SDK as a shared library" ${BUILD_SHARED_DEFAULT})
option(XRONOS_SDK_BUILD_TESTS "Build the tests" ${BUILD_TESTS_DEFAULT})
option(XRONOS_SDK_BUILD_EXAMPLES "Build the examples" ${BUILD_EXAMPLES_DEFAULT})
option(XRONOS_SDK_BUILD_DOCS "Build API documentation using Doxygen" ${BUILD_DOCS_DEFAULT})

option(XRONOS_SDK_ENABLE_DIAGRAMS "Enable exporting diagrams for visualization." ON)
option(XRONOS_SDK_ENABLE_TELEMETRY "Enable the recording of telemetry data (enable_telemetry() also needs to be called by the program)." ON)

if(XRONOS_SDK_INSTALL AND NOT XRONOS_SDK_BUILD_SHARED)
  message(FATAL_ERROR "Installing the Xronos SDK requires building with XRONOS_SDK_BUILD_SHARED=ON.")
endif()

set(XRONOS_LIB_WITH_GRAPH_EXPORTER ${XRONOS_SDK_ENABLE_DIAGRAMS})
set(XRONOS_LIB_WITH_OTEL ${XRONOS_SDK_ENABLE_TELEMETRY})
set(XRONOS_LIB_INSTALL OFF)
include(FetchContent)
FetchContent_Declare(
  xronos-lib
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/../lib"
  EXCLUDE_FROM_ALL TRUE
  FIND_PACKAGE_ARGS CONFIG
)
FetchContent_MakeAvailable(xronos-lib)

set(XRONOS_SDK_LIB_TYPE)
if(XRONOS_SDK_BUILD_SHARED)
  set(XRONOS_SDK_LIB_TYPE SHARED)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

configure_file(cmake/config.hh.in include/xronos/sdk/gen/config.hh @ONLY)

file(GLOB_RECURSE SDK_HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh" "${CMAKE_CURRENT_BINARY_DIR}/include/*.hh")

# xronos-sdk-public-headers carries the header file set; xronos-sdk (below)
# is the library consumers link.
add_library(xronos-sdk-public-headers INTERFACE)
target_sources(
  xronos-sdk-public-headers
  INTERFACE FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${SDK_HEADERS}
)
# The public SDK headers include the ABI headers (xronos/abi/...) and the
# value-boxing headers (xronos/value/...). Both components are header-only
# and ship as part of the SDK header package rather than as separate
# artifacts, so resolve their include directories here (they differ
# depending on whether xronos-lib was built from source or found as an
# installed package) and install their headers below. Plain paths are used
# instead of linking the xronos:: targets so that the exported
# xronos-sdkTargets do not depend on targets outside the SDK's export set.
if(xronos-lib_SOURCE_DIR)
  set(XRONOS_ABI_INCLUDE_DIR "${xronos-lib_SOURCE_DIR}/abi/include")
  set(XRONOS_VALUE_INCLUDE_DIR "${xronos-lib_SOURCE_DIR}/value/include")
else()
  get_target_property(XRONOS_ABI_INCLUDE_DIR xronos::xronos-abi INTERFACE_INCLUDE_DIRECTORIES)
  get_target_property(XRONOS_VALUE_INCLUDE_DIR xronos::xronos-value INTERFACE_INCLUDE_DIRECTORIES)
endif()
target_include_directories(xronos-sdk-public-headers
  INTERFACE $<BUILD_INTERFACE:${XRONOS_ABI_INCLUDE_DIR}> $<BUILD_INTERFACE:${XRONOS_VALUE_INCLUDE_DIR}>)
set_target_properties(xronos-sdk-public-headers PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

# The SDK borrows the ABI include path without linking xronos-abi, so the opt-out
# defined on that target does not reach SDK consumers (pybind, user apps); define
# it here too. See contract.hh.
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  target_compile_definitions(xronos-sdk-public-headers INTERFACE XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION)
endif()

# The SDK library: the (largely header-only) SDK glue plus the complete
# default implementation -- the shared backend, the default runtime, and the
# runtime-provider support -- in one artifact, shipped like the
# pre-ABI libxronos-sdk.so. It exports the SDK's public symbols plus the
# program-context factory (src/environment.cc) that the inline Environment
# constructors call.
add_library(xronos-sdk ${XRONOS_SDK_LIB_TYPE} src/environment.cc src/runtime_provider.cc)

target_link_libraries(
  xronos-sdk
  PUBLIC xronos-sdk-public-headers
  PRIVATE xronos::xronos-backend xronos::xronos-runtime-default xronos::xronos-util
)

# On Linux, restrict the exported symbols to the SDK's real public surface via
# a version script (an allowlist; `local: *` hides everything else). This is
# stronger than -Wl,--exclude-libs,ALL: it hides not only the statically linked
# internal archives but also the SDK's own inline vague-linkage spill (the
# runtime implementation, context/provider vtables and typeinfos). See
# xronos-sdk.map, and check-exported-symbols.sh for the enforced surface.
# Version scripts are a GNU ld feature; macOS expresses the SAME surface with
# -exported_symbols_list (exported_symbols.txt).
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(xronos-sdk PRIVATE -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/xronos-sdk.map)
  set_target_properties(xronos-sdk PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/xronos-sdk.map)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  target_link_options(xronos-sdk PRIVATE -Wl,-exported_symbols_list,${CMAKE_CURRENT_SOURCE_DIR}/exported_symbols.txt)
  set_target_properties(xronos-sdk PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/exported_symbols.txt)
endif()

target_compile_features(xronos-sdk PUBLIC cxx_std_20)
target_compile_options(xronos-sdk PRIVATE -Wall -Wextra -pedantic -Werror)
set_target_properties(xronos-sdk PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 1)

add_library(xronos::xronos-sdk ALIAS xronos-sdk)
add_library(xronos::xronos-sdk-public-headers ALIAS xronos-sdk-public-headers)

add_subdirectory(examples)

if(XRONOS_SDK_BUILD_TESTS)
  enable_testing()
endif()
add_subdirectory(test)
add_subdirectory(link-test)

if(XRONOS_SDK_BUILD_DOCS)
  add_subdirectory(docs)
endif()

if(XRONOS_SDK_INSTALL)
  include(GNUInstallDirs)

  # The public SDK headers include the xronos ABI headers (xronos/abi/value.hh)
  # and the value-boxing headers (xronos/value/boxing.hh). Bundle those headers
  # into the SDK's install tree so that the installed SDK package remains
  # self-contained and consumers do not need the xronos-lib package. Both
  # components are header-only, so headers are all we need.
  foreach(_xronos_header_lib IN ITEMS abi value)
    get_target_property(_xronos_lib_is_imported xronos::xronos-${_xronos_header_lib} IMPORTED)
    if(_xronos_lib_is_imported)
      get_target_property(_xronos_lib_include_dir xronos::xronos-${_xronos_header_lib} INTERFACE_INCLUDE_DIRECTORIES)
      list(GET _xronos_lib_include_dir 0 _xronos_lib_include_dir)
    else()
      set(_xronos_lib_include_dir "${xronos-lib_SOURCE_DIR}/${_xronos_header_lib}/include")
    endif()
    install(
      DIRECTORY "${_xronos_lib_include_dir}/xronos/${_xronos_header_lib}"
      DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/xronos")
  endforeach()

  install(
    TARGETS xronos-sdk xronos-sdk-public-headers
    EXPORT xronos-sdkTargets
    FILE_SET HEADERS
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

  install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
    ${CMAKE_CURRENT_SOURCE_DIR}/THIRD_PARTY_NOTICE
    DESTINATION "share/licenses/xronos-sdk"
  )
  install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/third-party-licenses
    DESTINATION "share/licenses/xronos-sdk"
  )

  # cpack configuration for exporting packages
  set(CPACK_GENERATOR "DEB;TGZ")
  set(CPACK_THREADS 8)
  set(CPACK_STRIP_FILES ON)
  SET(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SOURCE_DIR}/pkgs")

  set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
  set(CPACK_PACKAGE_VENDOR "Xronos, Inc.")
  set(CPACK_PACKAGE_CHECKSUM "SHA512")

  set(CPACK_VERBATIM_VARIABLES YES)

  set(CPACK_PACKAGE_CONTACT "christian@xronos.com")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Christian Menard <christian@xronos.com>")
  set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT) # use debian naming scheme for the deb package

  set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
  set(CPACK_DEBIAN_PACKAGE_GENERATE_SHLIBS ON)

  set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}-${CMAKE_SYSTEM_PROCESSOR}")

  set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)

  include(CPack)
endif()

set(LINT_FILES
  $<TARGET_PROPERTY:xronos-sdk-public-headers,HEADER_SET>
  ${CMAKE_CURRENT_SOURCE_DIR}/src/environment.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/runtime_provider.cc
  ${EXAMPLE_SOURCES}
)

set(FORMAT_FILES ${LINT_FILES} ${TEST_SOURCES} ${LINK_TEST_SOURCES})

add_custom_target(xronos-sdk-format
  COMMAND clang-format -i -style=file ${FORMAT_FILES}
  COMMAND_EXPAND_LISTS
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
add_custom_target(xronos-sdk-check-format
  COMMAND clang-format --dry-run --Werror -style=file ${FORMAT_FILES}
  COMMAND_EXPAND_LISTS
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

include(ProcessorCount)
ProcessorCount(NCORES)
add_custom_target(xronos-sdk-lint COMMAND
  echo ${LINT_FILES} | xargs -n1 -P${NCORES} clang-tidy --quiet -p ${CMAKE_CURRENT_BINARY_DIR}
  COMMAND_EXPAND_LISTS
)


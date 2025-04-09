set(XRONOS_OPENTELEMETRY_VERSION "1.17.0" CACHE STRING "Opentelemetry version")
set(XRONOS_OPENTELEMETRY_PROVIDER "module" CACHE STRING "opentelemetry provider (module|package|none)")

function(add_opentelemetry)
  include(FetchContent)
  # ensure that the variables set below overwrite the build options
  # opentelemetry
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  # set build options
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(WITH_STL "CXX${CMAKE_CXX_STANDARD}" CACHE STRING "Which version of the Standard Library for C++ to use")
  set(WITH_OTLP_GRPC ON)
  set(WITH_ABSEIL ON)
  set(WITH_BENCHMARK OFF)
  set(WITH_EXAMPLES OFF)
  set(WITH_DEPRECATED_SDK_FACTORY OFF)
  set(BUILD_TESTING OFF)
  set(BUILD_SHARED_LIBS OFF)
  # cloning may take a moment and this shows progress
  set(FETCHCONTENT_QUIET FALSE)
  FetchContent_Declare(
    opentelemetry-cpp
    GIT_REPOSITORY https://github.com/open-telemetry/opentelemetry-cpp.git
    GIT_TAG "v${XRONOS_OPENTELEMETRY_VERSION}"
    GIT_SHALLOW TRUE
    GIT_SUBMODULES ""
  )

  FetchContent_MakeAvailable(opentelemetry-cpp)

  # define alias targets so that it looks as if opentelemetry was installed
  add_library(opentelemetry-cpp::api ALIAS opentelemetry_api)
  add_library(opentelemetry-cpp::otlp_grpc_exporter ALIAS opentelemetry_exporter_otlp_grpc)
  add_library(opentelemetry-cpp::proto ALIAS opentelemetry_proto)
  add_library(opentelemetry-cpp::proto_grpc ALIAS opentelemetry_proto_grpc)
  add_library(opentelemetry-cpp::sdk ALIAS opentelemetry_sdk)
endfunction()

if(XRONOS_OPENTELEMETRY_PROVIDER STREQUAL "package")
  find_package(opentelemetry-cpp CONFIG REQUIRED)
elseif(XRONOS_OPENTELEMETRY_PROVIDER STREQUAL "module")
  add_opentelemetry()
endif()


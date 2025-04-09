set(XRONOS_GRPC_VERSION "v1.71.0" CACHE STRING "grpc version")
set(XRONOS_GRPC_PROVIDER "module" CACHE STRING "grpc provider (module|package|none)")

function(add_grpc)
  include(FetchContent)

  find_package(ZLIB REQUIRED)

  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(CMAKE_C_FLAGS "-w")
  set(CMAKE_CXX_FLAGS "-w")

  set(gRPC_BUILD_TESTS OFF)
  set(gRPC_BUILD_CODEGEN ON)
  set(gRPC_BUILD_GRPC_CPP_PLUGIN ON)
  set(gRPC_BUILD_CSHARP_EXT OFF)
  set(gRPC_BUILD_GRPC_CSHARP_PLUGIN OFF)
  set(gRPC_BUILD_GRPC_NODE_PLUGIN OFF)
  set(gRPC_BUILD_GRPC_OBJECTIVE_C_PLUGIN OFF)
  set(gRPC_BUILD_GRPC_PHP_PLUGIN OFF)
  set(gRPC_BUILD_GRPC_PYTHON_PLUGIN OFF)
  set(gRPC_BUILD_GRPC_RUBY_PLUGIN OFF)

  set(gRPC_BENCHMARK_PROVIDER "none" CACHE STRING "" FORCE)
  set(gRPC_ABSL_PROVIDER "package" CACHE STRING "" FORCE)
  set(gRPC_PROTOBUF_PROVIDER "package" CACHE STRING "" FORCE)
  set(gRPC_ZLIB_PROVIDER "package" CACHE STRING "" FORCE)

  FetchContent_Declare(
    gRPC
    GIT_REPOSITORY https://github.com/grpc/grpc
    GIT_TAG        "${XRONOS_GRPC_VERSION}"
    GIT_PROGRESS   TRUE
    GIT_SHALLOW    TRUE
    GIT_SUBMODULES_RECURSE FALSE
    GIT_SUBMODULES
      "third_party/cares"
      "third_party/boringssl-with-bazel"
      "third_party/re2"
    OVERRIDE_FIND_PACKAGE
  )
  FetchContent_MakeAvailable(gRPC)

  add_library(gRPC::grpc ALIAS grpc)
  add_library(gRPC::grpc++ ALIAS grpc++)
  add_executable(gRPC::grpc_cpp_plugin ALIAS grpc_cpp_plugin)
  # needed by opentelemetry
  set_target_properties(grpc_cpp_plugin PROPERTIES IMPORTED_LOCATION "\$<TARGET_FILE:gRPC::grpc_cpp_plugin>")
endfunction()

if(XRONOS_GRPC_PROVIDER STREQUAL "package")
  find_package(gRPC CONFIG REQUIRED)
elseif(XRONOS_GRPC_PROVIDER STREQUAL "module")
  add_grpc()
endif()

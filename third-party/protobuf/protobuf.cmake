set(XRONOS_PROTOBUF_VERSION "v31.1" CACHE STRING "protobuf version")
set(XRONOS_PROTOBUF_PROVIDER "module" CACHE STRING "protobuf provider (module|package|none)")
option(XRONOS_PROTOBUF_INSTALL "Add protobuf to the install target" OFF)

function(add_protobuf)
  include(FetchContent)

  find_package(ZLIB REQUIRED)

  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  set(CMAKE_C_FLAGS "-w")
  set(CMAKE_CXX_FLAGS "-w")

  set(protobuf_INSTALL ${XRONOS_PROTOBUF_INSTALL})
  set(utf8_range_ENABLE_INSTALL ${XRONOS_PROTOBUF_INSTALL})
  set(protobuf_BUILD_TESTS OFF)
  set(protobuf_BUILD_CONFORMANCE OFF)
  set(protobuf_BUILD_EXAMPLES OFF)
  set(protobuf_BUILD_PROTOC_BINARIES ON)
  set(protobuf_WITH_ZLIB ON CACHE BOOL "" FORCE)
  set(protobuf_ABSL_PROVIDER "package" CACHE STRING "" FORCE)
  FetchContent_Declare(Protobuf
    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
    GIT_TAG        "${XRONOS_PROTOBUF_VERSION}"
    GIT_SHALLOW    TRUE
    GIT_SUBMODULES_RECURSE FALSE
    GIT_SUBMODULES ""
    OVERRIDE_FIND_PACKAGE
  )
  FetchContent_MakeAvailable(Protobuf)

  FetchContent_GetProperties(Protobuf SOURCE_DIR Protobuf_SOURCE_DIR)
  include(${Protobuf_SOURCE_DIR}/cmake/protobuf-generate.cmake)
endfunction()

if(XRONOS_PROTOBUF_PROVIDER STREQUAL "package")
  find_package(Protobuf CONFIG REQUIRED)
elseif(XRONOS_PROTOBUF_PROVIDER STREQUAL "module")
  add_protobuf()
endif()

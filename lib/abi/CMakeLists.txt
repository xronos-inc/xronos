file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.hh")

add_library(xronos-abi INTERFACE)
target_sources(
  xronos-abi
  INTERFACE FILE_SET HEADERS
  TYPE HEADERS
  BASE_DIRS $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  FILES ${HEADERS}
)

set_target_properties(xronos-abi PROPERTIES VERIFY_INTERFACE_HEADER_SETS ON)

# macOS is a single-compile convenience build, so relax the ABI domain checks
# (see contract.hh). This reaches targets that link xronos-abi; SDK consumers
# that only borrow the include path get the same define on
# xronos-sdk-public-headers (public/cpp-sdk/CMakeLists.txt).
if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
  target_compile_definitions(xronos-abi INTERFACE XRONOS_ABI_UNCHECKED_SINGLE_COMPILATION)
endif()

add_library(xronos::xronos-abi ALIAS xronos-abi)

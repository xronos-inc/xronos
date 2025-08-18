set(XRONOS_ABSL_VERSION "20250512.1" CACHE STRING "absl version")
set(XRONOS_ABSL_PROVIDER "module" CACHE STRING "absl provider (module|package|none)")

function(add_absl)
  include(FetchContent)

  set(CMAKE_POSITION_INDEPENDENT_CODE ON)

  set(ABSL_PROPAGATE_CXX_STD ON)
  set(ABSL_USE_SYSTEM_INCLUDES ON)
  FetchContent_Declare(
    absl
    GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
    GIT_TAG "${XRONOS_ABSL_VERSION}"
    OVERRIDE_FIND_PACKAGE
  )
  FetchContent_MakeAvailable(absl)
endfunction()

if(XRONOS_ABSL_PROVIDER STREQUAL "package")
  find_package(absl CONFIG REQUIRED)
elseif(XRONOS_ABSL_PROVIDER STREQUAL "module")
  add_absl()
endif()

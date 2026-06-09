set(EXPECTED_LITE_VERSION "v0.10.0" CACHE STRING "expected-lite version")

include(FetchContent)

function(add_expected_lite)
  FetchContent_Declare(
    expected-lite
    URL https://github.com/martinmoene/expected-lite/archive/refs/tags/${EXPECTED_LITE_VERSION}.tar.gz
  )
  FetchContent_MakeAvailable(expected-lite)
endfunction()

add_expected_lite()

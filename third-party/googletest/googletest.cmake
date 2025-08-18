set(GOOGLETEST_VERSION "1.17.0" CACHE STRING "GooglesTest version")

include(FetchContent)

function(add_googletest)
  set(BUILD_GMOCK OFF)
  set(INSTALL_GTEST OFF)

  FetchContent_Declare(
    googletest
    URL "https://github.com/google/googletest/archive/refs/tags/v${GOOGLETEST_VERSION}.tar.gz"
  )
  FetchContent_MakeAvailable(googletest)
endfunction()

add_googletest()

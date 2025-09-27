set(FMT_VERSION "12.0.0" CACHE STRING "{fmt} version")

include(FetchContent)

function(add_fmt)
  set(FMT_INSTALL ON)
  set(FMT_SYSTEM_HEADERS ON)
  FetchContent_Declare(
    fmt
    URL https://github.com/fmtlib/fmt/archive/refs/tags/${FMT_VERSION}.tar.gz
  )
  FetchContent_MakeAvailable(fmt)
endfunction()

add_fmt()

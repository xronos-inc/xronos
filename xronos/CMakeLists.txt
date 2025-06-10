cmake_minimum_required(VERSION 3.28)

project(
  ${SKBUILD_PROJECT_NAME}
  LANGUAGES CXX
  VERSION ${SKBUILD_PROJECT_VERSION})

# require C++ 20
set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Colorize compilation output
set(CMAKE_COLOR_DIAGNOSTICS ON)

# Generate compilation database for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(RUN_CLANG_TIDY "Enable building with clang-tidy" Off)
option(CLANG_TIDY_ERROR "clang-tidy warnings are errors" Off)
option(XRONOS_BINDINGS_RUN_CLANG_TIDY "Run clang-tidy when compiling the bindings" OFF)
set(CLANG_TIDY_COMMAND "clang-tidy" CACHE STRING "The clang-tidy command to use")

if(XRONOS_BINDINGS_RUN_CLANG_TIDY)
  find_program(CLANG_TIDY ${CLANG_TIDY_COMMAND} REQUIRED)
  if(CLANG_TIDY_ERROR)
    set(CLANG_TIDY_BASE_COMMAND ${CLANG_TIDY_COMMAND}; -warnings-as-errors=*;)
  else()
    set(CLANG_TIDY_BASE_COMMAND ${CLANG_TIDY_COMMAND};)
  endif()
  set(XRONOS_CLANG_TIDY_COMMAND
    "${CLANG_TIDY_BASE_COMMAND};-header-filter=xronos/\(.*\)\\.hh")
endif()

include(FetchContent)
FetchContent_Declare(
  xronos-sdk
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/../cpp-sdk"
  EXCLUDE_FROM_ALL TRUE
  FIND_PACKAGE_ARGS CONFIG
)
FetchContent_MakeAvailable(xronos-sdk)
FetchContent_Declare(
  xronos-lib
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/../cpp-lib"
  EXCLUDE_FROM_ALL TRUE
  FIND_PACKAGE_ARGS CONFIG
)
FetchContent_MakeAvailable(xronos-lib)

add_compile_options(-Wall -Wextra -pedantic -Werror)

add_subdirectory(cpp)

install(TARGETS _cpp_sdk DESTINATION ${SKBUILD_PROJECT_NAME})

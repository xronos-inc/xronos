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

# Generate compilation database
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  xronos-sdk
  SOURCE_DIR "${PROJECT_SOURCE_DIR}/../cpp-sdk"
  EXCLUDE_FROM_ALL TRUE
  FIND_PACKAGE_ARGS CONFIG
)
FetchContent_MakeAvailable(xronos-sdk)

add_compile_options(-Wall -Wextra -pedantic -Werror)

find_package(Python 3.10.0...<3.15.0 REQUIRED COMPONENTS Interpreter
  Development.Module)
find_package(pybind11 CONFIG REQUIRED)

python_add_library(_cpp_sdk MODULE "${CMAKE_CURRENT_SOURCE_DIR}/cpp/cpp_sdk.cc" WITH_SOABI)
target_link_libraries(_cpp_sdk PRIVATE xronos::xronos-sdk pybind11::headers)

target_include_directories(_cpp_sdk PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")
set_target_properties(_cpp_sdk PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN YES
)

install(TARGETS _cpp_sdk DESTINATION ${SKBUILD_PROJECT_NAME})

add_custom_target(xronos-py-bindings-format
  COMMAND clang-format -i -style=file $<TARGET_PROPERTY:_cpp_sdk,SOURCES>
  COMMAND_EXPAND_LISTS
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)
add_custom_target(xronos-py-bindings-check-format
  COMMAND clang-format --dry-run --Werror -style=file $<TARGET_PROPERTY:_cpp_sdk,SOURCES>
  COMMAND_EXPAND_LISTS
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

add_custom_target(xronos-py-bindings-lint COMMAND
  clang-tidy --quiet -p ${CMAKE_CURRENT_BINARY_DIR} $<TARGET_PROPERTY:_cpp_sdk,SOURCES>
  COMMAND_EXPAND_LISTS
)

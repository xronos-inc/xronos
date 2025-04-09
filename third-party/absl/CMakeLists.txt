cmake_minimum_required(VERSION 3.28)

project("absl")

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ABSL_ENABLE_INSTALL ON)
include(absl.cmake)

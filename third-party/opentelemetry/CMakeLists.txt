cmake_minimum_required(VERSION 3.28)

project("opentelemetry")

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OPENTELEMETRY_INSTALL ON)
include(opentelemetry.cmake)

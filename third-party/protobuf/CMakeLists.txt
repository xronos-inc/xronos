cmake_minimum_required(VERSION 3.28)

project("protobuf")

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(XRONOS_PROTOBUF_INSTALL ON)
include(protobuf.cmake)

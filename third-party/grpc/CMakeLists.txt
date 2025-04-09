cmake_minimum_required(VERSION 3.28)

project("grpc")

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(gRPC_INSTALL ON CACHE BOOL "" FORCE)
include(grpc.cmake)

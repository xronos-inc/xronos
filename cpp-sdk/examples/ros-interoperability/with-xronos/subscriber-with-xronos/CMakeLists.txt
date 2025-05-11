# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.28)
project(ros_xronos_example_subscriber VERSION 0.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard is cached for visibility in external tools." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_executable(ros_xronos_example_subscriber src/subscriber.cpp)

include(FetchContent)
FetchContent_Declare(
  xronos-sdk
  GIT_REPOSITORY https://github.com/xronos-inc/xronos.git
  GIT_TAG v0.3.0
  SOURCE_SUBDIR cpp-sdk
)
FetchContent_MakeAvailable(xronos-sdk)

target_link_libraries(ros_xronos_example_subscriber xronos-sdk)
ament_target_dependencies(ros_xronos_example_subscriber rclcpp std_msgs)

install(TARGETS ros_xronos_example_subscriber DESTINATION lib/${PROJECT_NAME})

ament_auto_package()

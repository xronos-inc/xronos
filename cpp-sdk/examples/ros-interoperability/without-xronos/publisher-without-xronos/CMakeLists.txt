cmake_minimum_required(VERSION 3.28)
project(ros_xronos_example_publisher VERSION 0.0.0 LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_executable(ros_xronos_example_publisher src/publisher.cpp)

ament_target_dependencies(ros_xronos_example_publisher rclcpp std_msgs)

install(TARGETS ros_xronos_example_publisher DESTINATION lib/${PROJECT_NAME})

ament_auto_package()

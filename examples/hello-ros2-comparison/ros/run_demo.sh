#!/bin/bash

# docker run --rm -it -v "$PWD":/ros2_ws -w /ros2_ws osrf/ros:rolling-desktop ./entrypoint.sh

mkdir -p ./ros2_ws

docker run --rm -it --mount type=bind,source=./ros2_ws,target=/ros2_ws --mount type=bind,source=./hello,target=/ros2_ws/hello --mount type=bind,source=./entrypoint.sh,target=/entrypoint.sh -w /ros2_ws osrf/ros:rolling-desktop /entrypoint.sh

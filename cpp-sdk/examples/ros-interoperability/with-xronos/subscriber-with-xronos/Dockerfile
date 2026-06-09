# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

FROM ros:jazzy@sha256:6513503d0b10e919fbe8134981d4f9d19b5c1f9b045b87a9fe3b0b9e03e7c2a9

WORKDIR /root/ros2_ws/src

COPY . subscriber/

WORKDIR /root/ros2_ws/

SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=build source /opt/ros/jazzy/setup.bash && \
  rosdep install -i --from-path src --rosdistro jazzy -y && \
  colcon build --packages-select ros_xronos_example_subscriber

COPY ./run.sh run.sh

ENTRYPOINT [ "./run.sh" ]

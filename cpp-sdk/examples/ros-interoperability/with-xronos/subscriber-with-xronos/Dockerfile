# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

FROM ros:jazzy

WORKDIR /root/ros2_ws/src

COPY . subscriber/

WORKDIR /root/ros2_ws/

SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=build source /opt/ros/jazzy/setup.bash && \
  rosdep install -i --from-path src --rosdistro jazzy -y && \
  colcon build --packages-select ros_xronos_example_subscriber

COPY ./run.sh run.sh

ENTRYPOINT [ "./run.sh" ]

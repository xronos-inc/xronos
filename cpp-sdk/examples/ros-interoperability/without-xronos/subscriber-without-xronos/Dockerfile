FROM ros:jazzy@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f

WORKDIR /root/ros2_ws/src

COPY . subscriber/

WORKDIR /root/ros2_ws/

SHELL ["/bin/bash", "-c"]

RUN --mount=type=cache,target=build source /opt/ros/jazzy/setup.bash && \
  rosdep install -i --from-path src --rosdistro jazzy -y && \
  colcon build --packages-select ros_xronos_example_subscriber

COPY ./run.sh run.sh

ENTRYPOINT [ "./run.sh" ]

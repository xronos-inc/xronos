#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash

set -euo pipefail

ros2 run ros_xronos_example_subscriber ros_xronos_example_subscriber

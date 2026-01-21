# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(package="hello", executable="hello", name="hello"),
            launch_ros.actions.Node(
                package="hello", executable="printer", name="printer"
            ),
        ]
    )

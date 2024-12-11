# SPDX-FileCopyrightText: © 2024 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from pose import Actuator, Pose

P_MOULD = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 130,
        Actuator.PITCH2_DEG: 0,
        Actuator.PITCH3_DEG: 0,
        Actuator.GRIPPER_ROT_DEG: 90,
    },
    name="Mould",
)

P_TOP = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 80,
        Actuator.PITCH2_DEG: 50,
        Actuator.PITCH3_DEG: 50,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Top",
)

P_UP = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 90,
        Actuator.PITCH2_DEG: 90,
        Actuator.PITCH3_DEG: 90,
        Actuator.GRIPPER_ROT_DEG: 270,
        Actuator.GRIPPER_DEG: 90,
    },
    name="Up",
)

P_YELLOW = Pose(
    {
        Actuator.ROT_Z_DEG: 65,
        Actuator.PITCH1_DEG: 22,
        Actuator.PITCH2_DEG: 64,
        Actuator.PITCH3_DEG: 56,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Yellow",
)

P_RED = Pose(
    {
        Actuator.ROT_Z_DEG: 117,
        Actuator.PITCH1_DEG: 19,
        Actuator.PITCH2_DEG: 66,
        Actuator.PITCH3_DEG: 56,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Red",
)

P_GREEN = Pose(
    {
        Actuator.ROT_Z_DEG: 136,
        Actuator.PITCH1_DEG: 66,
        Actuator.PITCH2_DEG: 20,
        Actuator.PITCH3_DEG: 29,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Green",
)

P_BLUE = Pose(
    {
        Actuator.ROT_Z_DEG: 44,
        Actuator.PITCH1_DEG: 66,
        Actuator.PITCH2_DEG: 20,
        Actuator.PITCH3_DEG: 28,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Blue",
)

P_LAYER_4 = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 72,
        Actuator.PITCH2_DEG: 49,
        Actuator.PITCH3_DEG: 13,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Stack Layer 4",
)

P_LAYER_3 = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 66,
        Actuator.PITCH2_DEG: 43,
        Actuator.PITCH3_DEG: 20,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Stack Layer 3",
)

P_LAYER_2 = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 63,
        Actuator.PITCH2_DEG: 34,
        Actuator.PITCH3_DEG: 30,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Stack Layer 2",
)

P_LAYER_1 = Pose(
    {
        Actuator.ROT_Z_DEG: 90,
        Actuator.PITCH1_DEG: 53,
        Actuator.PITCH2_DEG: 33,
        Actuator.PITCH3_DEG: 36,
        Actuator.GRIPPER_ROT_DEG: 270,
    },
    name="Stack Layer 1",
)

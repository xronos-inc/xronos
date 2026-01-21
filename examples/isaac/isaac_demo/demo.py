# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

import xronos

from .controller import CubeGeneratorController, RobotArmController
from .isaac import IsaacCubeGenerator, IsaacFranka, IsaacSim


def demo(enable_telemetry: bool) -> None:
    env = xronos.Environment()

    # reactors that interface with isaac sim
    isaac_sim = env.create_reactor("isaac_sim", IsaacSim)
    isaac_robot_arm = env.create_reactor("isaac_franka", IsaacFranka)
    isaac_cube_generator = env.create_reactor(
        "isaac_cube_generator", IsaacCubeGenerator
    )

    # isaac agnostic controllers
    robot_arm_controller = env.create_reactor(
        "robot_arm_controller", RobotArmController
    )
    cube_controller = env.create_reactor("cube_controller", CubeGeneratorController)

    env.connect(robot_arm_controller.target_arm_pose, isaac_robot_arm.target_arm_pose)
    env.connect(robot_arm_controller.open_gripper, isaac_robot_arm.open_gripper)
    env.connect(robot_arm_controller.close_gripper, isaac_robot_arm.close_gripper)
    env.connect(robot_arm_controller.cube_placed, cube_controller.cube_placed)

    env.connect(isaac_robot_arm.done_arm, robot_arm_controller.pose_reached)
    env.connect(isaac_robot_arm.done_gripper, robot_arm_controller.gripper_reached)

    env.connect(cube_controller.spawn_new_cube, isaac_cube_generator.spawn_new_cube)

    env.connect(isaac_sim.do_step, isaac_robot_arm.do_step)

    if enable_telemetry:
        env.enable_telemetry()

    env.execute()

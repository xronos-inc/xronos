# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

import math
from typing import Callable, Tuple

import numpy as np
import numpy.typing as npt
import xronos

MAX_CUBES = 20
STACK_HEIGHT = 5


class RobotArmController(xronos.Reactor):
    """Reactor implementing the high-level pick and place control."""

    # Output port for sending a new target arm pose.
    target_arm_pose = xronos.OutputPortDeclaration[
        Tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]]
    ]()
    # Output port indicating that the gripper should close.
    close_gripper = xronos.OutputPortDeclaration[None]()
    # Output port indicating that the gripper should open.
    open_gripper = xronos.OutputPortDeclaration[None]()
    # Output port indicating that a cube was successfully placed.
    cube_placed = xronos.OutputPortDeclaration[None]()

    # Input port that is expected to be set when the arm is in its target
    # position.
    pose_reached = xronos.InputPortDeclaration[None]()
    # Input port that is expected to be set when the gripper joints are in
    # their target position.
    gripper_reached = xronos.InputPortDeclaration[None]()

    _cubes_delivered = xronos.MetricDeclaration(description="Cubes Delivered")

    _GRIPPER_ORIENTATION = np.array([0.0, 0.0, 1.0, 0.0])

    def __init__(self) -> None:
        super().__init__()
        self._sequence_step = 0
        self._cubes_delivered_value = 0

    def __get_next_command(self) -> Tuple[npt.NDArray[np.floating], bool, bool]:
        pos_x = math.floor(self._cubes_delivered_value / STACK_HEIGHT) * 0.20 - 0.08
        pos_z = (self._cubes_delivered_value % STACK_HEIGHT) * 0.04

        pickup_sequence = [
            (np.array([0.6, 0.0, 0.1]), True, False),
            (np.array([0.6, 0.0, 0.01]), True, False),
            (np.array([0.6, 0.0, 0.01]), False, False),
            (np.array([pos_x, 0.55, pos_z + 0.1]), False, False),
            (np.array([pos_x, 0.55, pos_z + 0.015]), False, False),
            (np.array([pos_x, 0.55, pos_z + 0.015]), True, False),
            (np.array([pos_x, 0.55, pos_z + 0.1]), True, True),
        ]

        return pickup_sequence[self._sequence_step % len(pickup_sequence)]

    @xronos.reaction
    def control_step(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Reaction implementing the main control algorithm.

        The controller computes a sequence of steps. When both the arm and the
        gripper reached their target position, the controller proceeds to the
        next step in the sequence and sends new commands to the robot.
        """
        startup_trigger = interface.add_trigger(self.startup)
        pose_reached_trigger = interface.add_trigger(self.pose_reached)
        gripper_reached_trigger = interface.add_trigger(self.gripper_reached)

        target_arm_pose_effect = interface.add_effect(self.target_arm_pose)
        open_gripper_effect = interface.add_effect(self.open_gripper)
        close_gripper_effect = interface.add_effect(self.close_gripper)
        cube_placed_effect = interface.add_effect(self.cube_placed)
        cubes_delivered_metric = interface.add_effect(self._cubes_delivered)

        # Produces the next control output.
        def handler() -> None:
            if startup_trigger.is_present() or (
                pose_reached_trigger.is_present()
                and gripper_reached_trigger.is_present()
            ):
                pose, open_gripper, place_new_cube = self.__get_next_command()

                if self._cubes_delivered_value == MAX_CUBES:
                    END_POSE = np.array([0.6, 0.0, 0.3])
                    END_ORIENTATION = self._GRIPPER_ORIENTATION
                    target_arm_pose_effect.set((END_POSE, END_ORIENTATION))
                else:
                    target_arm_pose_effect.set((pose, self._GRIPPER_ORIENTATION))

                    if open_gripper:
                        open_gripper_effect.set(None)
                    else:
                        close_gripper_effect.set(None)

                    if place_new_cube:
                        cube_placed_effect.set(None)
                        self._cubes_delivered_value += 1
                        cubes_delivered_metric.record(self._cubes_delivered_value)

                    self._sequence_step += 1

        return handler


class CubeGeneratorController(xronos.Reactor):
    """Simple reactor controlling when cubes are generated.

    Sends a message on the `spawn_new_cube` port on startup or when receiving a
    message on `cube_placed` indicating that the previous cube was placed
    correctly.
    """

    cube_placed = xronos.InputPortDeclaration[None]()

    spawn_new_cube = xronos.OutputPortDeclaration[Tuple[float, float, float]]()

    CUBE_COLORS = (
        (0, 0.776, 0.957),
        (0.216, 0.322, 0.976),
        (0.42, 0.18, 0.937),
        (0.698, 0.188, 0.937),
        (0.89, 0.314, 0.984),
    )

    def __init__(self) -> None:
        super().__init__()
        self._num_cubes_placed = 0

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        spawn_new_cube_effect = interface.add_effect(self.spawn_new_cube)

        def handler() -> None:
            spawn_new_cube_effect.set(self.CUBE_COLORS[0])

        return handler

    @xronos.reaction
    def on_cube_placed(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.cube_placed)
        spawn_new_cube_effect = interface.add_effect(self.spawn_new_cube)

        def handler() -> None:
            self._num_cubes_placed += 1
            if self._num_cubes_placed < MAX_CUBES:
                spawn_new_cube_effect.set(
                    self.CUBE_COLORS[self._num_cubes_placed % STACK_HEIGHT]
                )

        return handler

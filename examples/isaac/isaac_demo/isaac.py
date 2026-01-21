# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard
# ruff: noqa E402
# mypy: disable-error-code="no-any-unimported"

import datetime
from typing import Callable, Tuple

import numpy as np
import numpy.typing as npt
import xronos
from isaacsim import SimulationApp  # type: ignore

simulation_app = SimulationApp({"headless": False})  # type: ignore

from isaacsim.core.api import World  # type: ignore
from isaacsim.core.api.objects import DynamicCuboid  # type: ignore
from isaacsim.core.utils.stage import add_reference_to_stage  # type: ignore
from isaacsim.core.prims import SingleXFormPrim  # type: ignore
from isaacsim.storage.native import get_assets_root_path  # type: ignore
from isaacsim.robot.manipulators.examples.franka import Franka  # type: ignore
from isaacsim.robot.manipulators.examples.franka.controllers import RMPFlowController  # type: ignore
from omni.isaac.core import SimulationContext  # type: ignore

isaac_world = World(stage_units_in_meters=1.0)
DESK_HEIGHT = 0.7947


class IsaacFranka(xronos.Reactor):
    """Reactor for interacting with a Franka robot arm.

    This reactor wraps an Isaac Franka robot arm and its controller. It
    instantiates the arm and controller on startup, receives control input via
    the input ports, and forwards the control commands to the underlying Isaac
    object.

    The reactor also sets the outputs `done_arm` or `done_gripper` when the
    target position is reached.
    """

    # Input port for setting a new arm target pos
    target_arm_pose = xronos.InputPortDeclaration[
        Tuple[npt.NDArray[np.floating], npt.NDArray[np.floating]]
    ]()
    # Input port for opening the gripper.
    open_gripper = xronos.InputPortDeclaration[None]()
    # Input port for closing the gripper.
    close_gripper = xronos.InputPortDeclaration[None]()

    # Input port for triggering a control step.
    do_step = xronos.InputPortDeclaration[None]()

    # Output port that is set when the arm reaches its target position.
    done_arm = xronos.OutputPortDeclaration[None]()
    # Output port that is set when the gripper joints reach their target
    # position.
    done_gripper = xronos.OutputPortDeclaration[None]()

    # metrics:
    _arm_distance = xronos.MetricDeclaration(
        description="Distance of the arm to the desired position", unit="m"
    )
    _gripper_distance = xronos.MetricDeclaration(
        description="Distance of the gripper joints to the desired position"
    )
    _arm_position_reached = xronos.MetricDeclaration(description="Arm position reached")
    _gripper_position_reached = xronos.MetricDeclaration(
        description="Gripper position reached"
    )

    _ARM_VELOCITY_TOLERANCE = 0.000025
    _ARM_ORIENTATION_TOLERANCE = 1.4425

    _GRIPPER_CLOSED_POSITION = (0.019, 0.019)
    _GRIPPER_OPEN_POSITION = (0.04, 0.04)
    _GRIPPER_POSITION_TOLERANCE = 0.0015

    def __init__(self) -> None:
        super().__init__()

        self._target_position = np.array([0.0, 0.0, 0.0])
        self._last_position = np.array([0.0, 0.0, 0.0])
        self._target_orientation = np.array([0.0, 1.0, 0.0, 0.0])
        self._gripper_target_position = self._GRIPPER_OPEN_POSITION

        self._franka = Franka(
            prim_path="/Franka", name="manipulator", position=[0, 0, DESK_HEIGHT]
        )
        self._controller = RMPFlowController(
            name="controller", robot_articulation=self._franka
        )
        isaac_world.scene.add(self._franka)
        isaac_world.reset()

    @xronos.reaction
    def on_do_step_arm(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Update the `arm_done` output for each control step."""

        interface.add_trigger(self.do_step)
        done_arm_effect = interface.add_effect(self.done_arm)
        distance_metric = interface.add_effect(self._arm_distance)
        position_reached_metric = interface.add_effect(self._arm_position_reached)

        def handler() -> None:
            position, orientation = self._franka.end_effector.get_world_pose()
            velocity = np.linalg.norm(
                np.array(self._last_position) - np.array(position)
            )
            self._last_position = position
            distance_orientation = np.linalg.norm(
                np.array(orientation) - np.array(self._target_orientation)
            )

            distance_metric.record(
                np.linalg.norm(self._target_position - position).item()
            )

            if (
                velocity < self._ARM_VELOCITY_TOLERANCE
                and distance_orientation < self._ARM_ORIENTATION_TOLERANCE
            ):
                done_arm_effect.set(None)
                position_reached_metric.record(1)
            else:
                position_reached_metric.record(0)

        return handler

    @xronos.reaction
    def on_do_step_gripper(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Update the `gripper_done` output for each control step."""

        interface.add_trigger(self.do_step)
        done_gripper_effect = interface.add_effect(self.done_gripper)
        distance_metric = interface.add_effect(self._gripper_distance)
        position_reached_metric = interface.add_effect(self._gripper_position_reached)

        def handler() -> None:
            current_gripper_position = self._franka.gripper.get_joint_positions()
            distance = np.linalg.norm(
                current_gripper_position - self._gripper_target_position
            )
            distance_metric.record(distance.item())
            if distance < self._GRIPPER_POSITION_TOLERANCE:
                done_gripper_effect.set(None)
                position_reached_metric.record(1)
            else:
                position_reached_metric.record(0)

        return handler

    @xronos.reaction
    def on_gripper_command(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Receive a new gripper command."""

        open_gripper_trigger = interface.add_trigger(self.open_gripper)
        close_gripper_trigger = interface.add_trigger(self.close_gripper)

        def handler() -> None:
            if open_gripper_trigger.is_present():
                self._gripper_target_position = self._GRIPPER_OPEN_POSITION
                self._franka.gripper.open()
            elif close_gripper_trigger.is_present():
                self._gripper_target_position = self._GRIPPER_CLOSED_POSITION
                self._franka.gripper.close()

        return handler

    @xronos.reaction
    def do_arm_control(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Handle the arm control for every step and receive target pose (if set)."""

        _ = interface.add_trigger(self.do_step)
        target_arm_pose_trigger = interface.add_trigger(self.target_arm_pose)

        def handler() -> None:
            if target_arm_pose_trigger.is_present():
                position, orientation = target_arm_pose_trigger.get()
                self._target_position = position + np.array([0.0, 0.0, DESK_HEIGHT])
                self._target_orientation = orientation

            actions = self._controller.forward(
                target_end_effector_position=self._target_position,
                target_end_effector_orientation=self._target_orientation,
            )
            self._franka.apply_action(actions)

        return handler


class IsaacCubeGenerator(xronos.Reactor):
    """Reactor responsible for spawning new cubes in the simulated world."""

    spawn_new_cube = xronos.InputPortDeclaration[Tuple[float, float, float]]()

    def __init__(self) -> None:
        super().__init__()
        self._num_cubes = 0

    @xronos.reaction
    def on_spawn_new_cube(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        spawn_new_cube_trigger = interface.add_trigger(self.spawn_new_cube)

        # Places the next cube on the table.
        def handler() -> None:
            cube = DynamicCuboid(
                prim_path=f"/cube_{self._num_cubes}",
                name=f"cube_{self._num_cubes}",
                translation=np.array([0.6, 0, DESK_HEIGHT + 0.1]),
                orientation=np.array([0, 0, 0, 0]),
                size=0.04,
                mass=0.01,
                density=64e-8,
                color=np.array(spawn_new_cube_trigger.get()),
            )

            isaac_world.scene.add(cube)
            self._num_cubes += 1

        return handler


class IsaacSim(xronos.Reactor):
    """Reactor responsible for managing the interaction with Isaac Sim.

    Initializes the basic scene and calls `isaac_world.step()` in intervals matching
    dt of Isaac's physics engine.
    """

    _timer = xronos.PeriodicTimerDeclaration(
        period=datetime.timedelta(seconds=SimulationContext.instance().get_physics_dt())
    )
    do_step = xronos.OutputPortDeclaration[None]()

    def __init__(self) -> None:
        super().__init__()

        isaac_world.scene.add_default_ground_plane()
        scene_path = "/Isaac/Props/Mounts/ThorlabsTable/table_instanceable.usd"
        assets_root_path = get_assets_root_path()

        if assets_root_path is None:
            print("Could not find Isaac Sim assets folder")
        else:
            add_reference_to_stage(assets_root_path + scene_path, "/table1")
            isaac_world.scene.add(
                SingleXFormPrim(
                    prim_path="/table1", name="tab1", position=[0, 0, DESK_HEIGHT]
                )
            )
            add_reference_to_stage(assets_root_path + scene_path, "/table2")
            isaac_world.scene.add(
                SingleXFormPrim(
                    prim_path="/table2", name="tab2", position=[0, 0.751, DESK_HEIGHT]
                )
            )

        isaac_world.reset()

    @xronos.reaction
    def advance_simulation(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Calls `isaac_world.step()` and triggers the `do_step` output port."""
        interface.add_trigger(self._timer)
        do_step = interface.add_effect(self.do_step)
        trigger_shutdown = interface.add_effect(self.shutdown)

        # Advances the simulation and produces the do_step output, which is used
        # by the other isaac reactors, to provide new control input into the simulator.
        def handler() -> None:
            if not simulation_app.is_running():
                trigger_shutdown.trigger_shutdown()
                print("Simulation app has stopped terminating the program!")
            else:
                isaac_world.step(render=True)
                do_step.set(None)

        return handler

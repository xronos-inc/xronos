# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import argparse
import datetime
import time
from typing import Callable, Optional, cast

import xronos
from parse import Result, parse  # type: ignore

import pose_constants
from Arm_Lib import Arm_Device
from pose import Actuator, Pose, Trajectory
from user_interface import UserInterface


class ArmControl(xronos.Reactor):
    _sample_timer = xronos.TimerDeclaration()
    _in_position_event = xronos.InternalEventDeclaration[bool]()
    new_trajectory = xronos.InputPortDeclaration[Trajectory]()
    trajectory_completed = xronos.OutputPortDeclaration[bool]()

    def __init__(self, device: Arm_Device, verbose: bool = False) -> None:
        super().__init__()
        self._sample_timer.offset = datetime.timedelta(milliseconds=100)
        self._sample_timer.period = datetime.timedelta(milliseconds=100)

        self.device = device
        self.position = Pose()  # state: current pose (updated periodically)
        self.was_in_position = True  # state: was previously in position?
        self.goal = Pose()  # state: goal pose
        self.trajectory = Trajectory([])  # state: current trajectory
        self.verbose = verbose

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            if not self.in_position():
                print("Warning: shutting down before robot reached goal position.")

        return handler

    @xronos.reaction
    def on_new_trajectory(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        new_trajectory_trigger = interface.add_trigger(self.new_trajectory)

        def handler() -> None:
            self.trajectory = new_trajectory_trigger.get()
            next_pose = self.trajectory.next_pose()
            if self.verbose:
                print(f"Received trajectory, setting next_pose={next_pose}")
            if next_pose:
                self.goal = next_pose

        return handler

    @xronos.reaction
    def on_sample_timer(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self._sample_timer)
        in_position_effect = interface.add_effect(self._in_position_event)

        # update position and schedule action if state of in_position changes
        def handler() -> None:
            self.read_pose()
            in_position = self.in_position()
            distance = Pose.delta(self.goal, self.position)
            if self.verbose:
                print(f"Distance: {distance}")

            if in_position != self.was_in_position:
                self.was_in_position = in_position
                in_position_effect.schedule(in_position)

        return handler

    @xronos.reaction
    def on_in_position_change(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        in_position_trigger = interface.add_trigger(self._in_position_event)
        trajectory_completed_effect = interface.add_effect(self.trajectory_completed)

        def handler() -> None:
            if not in_position_trigger.get():
                if self.verbose:
                    print(f"Goal distance: {(Pose.delta(self.goal, self.position))}")

                self.goal.validate()
                for actuator, pos_deg in self.goal.positions.items():
                    self.move_actuator(actuator, pos_deg)
            else:
                # Currrent pose goal reached. Check if more poses are in the trajectory.
                next_pose = self.trajectory.next_pose()
                if next_pose:
                    self.goal = next_pose
                else:
                    trajectory_completed_effect.set(True)

        return handler

    def move_actuator(
        self, actuator: Actuator, pos_deg: int, move_time_ms: int = 1500
    ) -> None:
        """Move a single actuator."""
        Pose.validate_actuator(actuator, pos_deg)
        device.Arm_serial_servo_write(actuator.value, pos_deg, move_time_ms)
        time.sleep(0.01)

    def in_position(self) -> bool:
        """Is the robot currently in the goal position?"""
        for actuator in Actuator:
            goal = self.goal[actuator]
            position = self.position[actuator]
            if goal:
                if (
                    not position
                    or abs(goal - position) > Pose.in_position_tolerance_deg
                ):
                    return False
        return True

    def read_pose(self) -> Pose:
        """Read the pose of the arm and store in state."""
        for actuator in Actuator:
            value = self.device.Arm_serial_servo_read(actuator.value)
            if value:
                self.position[actuator] = value
            else:
                print("Warning: failed to read actuator position " + actuator.name)
        return self.position


def user_input_parser(cmd: str) -> Trajectory | None | KeyboardInterrupt:
    if cmd == "help":
        print(
            "CLI to robot arm controller."
            + " Commands: help, moveto [<green/red/blue/yellow/init> ...]."
            + " Exit with `exit` or Ctrl+C"
        )
        return None
    elif cmd == "exit":
        return KeyboardInterrupt()

    r = parse("moveto {}", cmd)
    trajectory = list[Pose]()
    if not isinstance(r, Result):
        print(f"Could not parse command `{cmd}`")
        return None

    goals = cast(list[str], r[0].split(" "))  # type: ignore[reportUnknownMemberType]
    for goal in goals:
        pose = goal_to_pose(goal)
        if pose:
            trajectory.append(pose)
    if len(trajectory) > 0:
        return Trajectory(trajectory)
    else:
        return None


def goal_to_pose(goal: str) -> Pose | None:
    pose: Optional[Pose] = None
    if goal == "green":
        pose = pose_constants.P_GREEN
    elif goal == "red":
        pose = pose_constants.P_RED
    elif goal == "blue":
        pose = pose_constants.P_BLUE
    elif goal == "yellow":
        pose = pose_constants.P_YELLOW
    elif goal == "init":
        pose = pose_constants.P_UP
    elif goal == "top":
        pose = pose_constants.P_TOP
    else:
        print(f"Unknown pose `{goal}`")
    return pose


def main(device: Arm_Device, enable_tracing: bool) -> None:
    env = xronos.Environment()
    arm = env.create_reactor("arm_control", ArmControl, device)
    ui = env.create_reactor(
        "ui", UserInterface[Trajectory], blocking=True, parser=user_input_parser
    )
    env.connect(ui.output, arm.new_trajectory)
    env.connect(arm.trajectory_completed, ui.unblock)

    if enable_tracing:
        print("Enabling tracing.")
        env.enable_tracing()

    env.execute()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Yahboom robot arm controller")
    parser.add_argument(
        "--trace",
        action="store_true",
        help="Enable the xronos tracing feature.",
    )
    args = parser.parse_args()

    device = Arm_Device()
    time.sleep(0.1)
    main(device, enable_tracing=args.trace)

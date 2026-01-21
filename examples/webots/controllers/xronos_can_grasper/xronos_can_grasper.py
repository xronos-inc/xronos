# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# mypy: disable-error-code="no-any-unimported"
# pyright: reportUnknownMemberType=false, reportUnknownVariableType=false
# pyright: reportUnknownParameterType=false, reportUnknownArgumentType=false

import datetime
import math
import sys
from typing import Callable

import controller as webots  # type: ignore [import-not-found]
import xronos

TIME_STEP = 32


class Simulator(xronos.Reactor):
    done_step = xronos.OutputPortDeclaration[None]()
    __step_timer = xronos.ProgrammableTimerDeclaration[None]()
    __done_step_event = xronos.PhysicalEventDeclaration[None]()

    def __init__(self, robot: webots.Robot) -> None:
        super().__init__()
        self.__robot = robot

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.startup)
        step_effect = interface.add_effect(self.__step_timer)

        return lambda: step_effect.schedule(None, datetime.timedelta(0))

    @xronos.reaction
    def on_step(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.__step_timer)
        shutdown_effect = interface.add_effect(self.shutdown)

        def handler() -> None:
            if self.__robot.step(TIME_STEP) != -1:
                self.__done_step_event.trigger(None)
            else:
                shutdown_effect.trigger_shutdown()

        return handler

    @xronos.reaction
    def on_done_step(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.__done_step_event)
        step_effect = interface.add_effect(self.__step_timer)
        done_step_effect = interface.add_effect(self.done_step)

        def handler() -> None:
            step_effect.schedule(None)
            done_step_effect.set(None)

        return handler


class Motor(xronos.Reactor):
    step = xronos.InputPortDeclaration[None]()
    current_position = xronos.OutputPortDeclaration[float]()
    target_position = xronos.InputPortDeclaration[float]()

    _current_position = xronos.MetricDeclaration("current position")
    _target_position = xronos.MetricDeclaration("target position")

    def __init__(self, robot: webots.Robot, speed: float | None = None) -> None:
        super().__init__()
        self.add_attribute("device_type", "motor")

        self.__motor = robot.getDevice(self.name)
        if speed is not None:
            self.__motor.setVelocity(speed)

        self.__sensor = robot.getDevice(f"{self.name}_sensor")
        self.__sensor.enable(TIME_STEP)

    @xronos.reaction
    def on_step(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.step)
        position_effect = interface.add_effect(self.current_position)
        metric_effect = interface.add_effect(self._current_position)

        def handler() -> None:
            position: float = self.__sensor.getValue()
            position_effect.set(position)
            metric_effect.record(position)

        return handler

    @xronos.reaction
    def on_target_position(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        position_trigger = interface.add_trigger(self.target_position)
        metric_effect = interface.add_effect(self._target_position)

        def handler() -> None:
            position = position_trigger.get()
            self.__motor.setPosition(position)
            metric_effect.record(position)

        return handler


class DistanceSensor(xronos.Reactor):
    step = xronos.InputPortDeclaration[None]()
    out_current_distance = xronos.OutputPortDeclaration[float]()
    _current_distance = xronos.MetricDeclaration("current distance")

    def __init__(self, robot: webots.Robot) -> None:
        super().__init__()
        self.__sensor = robot.getDevice("distance sensor")
        self.__sensor.enable(TIME_STEP)
        self.add_attribute("device_type", "distance sensor")

    @xronos.reaction
    def on_step(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.step)
        distance_effect = interface.add_effect(self.out_current_distance)
        metric_effect = interface.add_effect(self._current_distance)

        def handler() -> None:
            distance: float = self.__sensor.getValue()
            distance_effect.set(distance)
            metric_effect.record(distance)

        return handler


class Controller(xronos.Reactor):
    distance = xronos.InputPortDeclaration[float]()

    hand_grasp = xronos.OutputPortDeclaration[None]()
    hand_release = xronos.OutputPortDeclaration[None]()
    hand_is_grasped = xronos.InputPortDeclaration[None]()
    hand_is_released = xronos.InputPortDeclaration[None]()

    arm_rotate = xronos.OutputPortDeclaration[None]()
    arm_rotate_back = xronos.OutputPortDeclaration[None]()
    arm_is_rotated = xronos.InputPortDeclaration[None]()
    arm_is_back = xronos.InputPortDeclaration[None]()

    DISTANCE_THRESHOLD = 500

    def __init__(self) -> None:
        super().__init__()
        self.__waiting = True

    @xronos.reaction
    def on_distance(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        distance_trigger = interface.add_trigger(self.distance)
        grasp_effect = interface.add_effect(self.hand_grasp)

        def handler() -> None:
            if self.__waiting and distance_trigger.get() < self.DISTANCE_THRESHOLD:
                print("Grasping can")
                self.__waiting = False
                grasp_effect.set(None)

        return handler

    @xronos.reaction
    def on_hand_is_grasped(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        _ = interface.add_trigger(self.hand_is_grasped)
        rotate_effect = interface.add_effect(self.arm_rotate)

        def handler() -> None:
            print("Rotating arm")
            rotate_effect.set(None)

        return handler

    @xronos.reaction
    def on_arm_is_rotated(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        _ = interface.add_trigger(self.arm_is_rotated)
        release_effect = interface.add_effect(self.hand_release)

        def handler() -> None:
            print("Releasing can")
            release_effect.set(None)

        return handler

    @xronos.reaction
    def on_hand_is_released(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        _ = interface.add_trigger(self.hand_is_released)
        rotate_back_effect = interface.add_effect(self.arm_rotate_back)

        def handler() -> None:
            print("Rotating arm back")
            rotate_back_effect.set(None)

        return handler

    @xronos.reaction
    def on_arm_is_back(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.arm_is_back)

        def handler() -> None:
            print("Waiting for can")
            self.__waiting = True

        return handler


class Hand(xronos.Reactor):
    step = xronos.InputPortDeclaration[None]()
    grasp = xronos.InputPortDeclaration[None]()
    release = xronos.InputPortDeclaration[None]()

    is_grasped = xronos.OutputPortDeclaration[None]()
    is_released = xronos.OutputPortDeclaration[None]()

    GRASP_POS = 0.80
    RELEASE_POS = 0.05

    def __init__(self, robot: webots.Robot) -> None:
        super().__init__()
        self.__fingers = [
            self.create_reactor("finger_1_joint_1", Motor, robot),
            self.create_reactor("finger_2_joint_1", Motor, robot),
            self.create_reactor("finger_middle_joint_1", Motor, robot),
        ]
        for finger in self.__fingers:
            self.connect(self.step, finger.step)
        self.__is_moving = False

    @xronos.reaction
    def on_position(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        finger_position_triggers = [
            interface.add_trigger(finger.current_position) for finger in self.__fingers
        ]
        is_grasped_effect = interface.add_effect(self.is_grasped)
        is_released_effect = interface.add_effect(self.is_released)

        def handler() -> None:
            if self.__is_moving and all(
                [
                    pos.is_present()
                    and math.isclose(pos.get(), self.GRASP_POS, abs_tol=0.02)
                    for pos in finger_position_triggers
                ]
            ):
                self.__is_moving = False
                is_grasped_effect.set(None)
            elif self.__is_moving and all(
                [
                    pos.is_present()
                    and math.isclose(pos.get(), self.RELEASE_POS, abs_tol=0.02)
                    for pos in finger_position_triggers
                ]
            ):
                self.__is_moving = False
                is_released_effect.set(None)

        return handler

    @xronos.reaction
    def on_grasp(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.grasp)
        finger_effects = [
            interface.add_effect(finger.target_position) for finger in self.__fingers
        ]

        def handler() -> None:
            self.__is_moving = True
            for finger in finger_effects:
                finger.set(self.GRASP_POS)

        return handler

    @xronos.reaction
    def on_release(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.release)
        finger_effects = [
            interface.add_effect(finger.target_position) for finger in self.__fingers
        ]

        def handler() -> None:
            self.__is_moving = True
            for finger_reset in finger_effects:
                finger_reset.set(self.RELEASE_POS)

        return handler


class Arm(xronos.Reactor):
    step = xronos.InputPortDeclaration[None]()
    rotate = xronos.InputPortDeclaration[None]()
    rotate_back = xronos.InputPortDeclaration[None]()

    is_rotated = xronos.OutputPortDeclaration[None]()
    is_back = xronos.OutputPortDeclaration[None]()

    ROTATED_POSITIONS = (-1.88, -2.14, -2.38, -1.51)
    NEUTRAL_POSITIONS = (0.0, 0.0, 0.0, 0.0)

    def __init__(self, robot: webots.Robot, speed: float) -> None:
        super().__init__()
        self.__joints = [
            self.create_reactor("shoulder_lift_joint", Motor, robot, speed),
            self.create_reactor("elbow_joint", Motor, robot, speed),
            self.create_reactor("wrist_1_joint", Motor, robot, speed),
            self.create_reactor("wrist_2_joint", Motor, robot, speed),
        ]
        for joint in self.__joints:
            self.connect(self.step, joint.step)

        self.__is_moving = False

    @xronos.reaction
    def on_position(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        joint_position_triggers = [
            interface.add_trigger(joint.current_position) for joint in self.__joints
        ]
        is_rotated_effect = interface.add_effect(self.is_rotated)
        is_back_effect = interface.add_effect(self.is_back)

        def handler() -> None:
            if self.__is_moving and all(
                [
                    pos.is_present() and math.isclose(pos.get(), target, abs_tol=0.1)
                    for pos, target in zip(
                        joint_position_triggers, self.ROTATED_POSITIONS
                    )
                ]
            ):
                self.__is_moving = False
                is_rotated_effect.set(None)
            elif self.__is_moving and all(
                [
                    pos.is_present() and math.isclose(pos.get(), target, abs_tol=0.02)
                    for pos, target in zip(
                        joint_position_triggers, self.NEUTRAL_POSITIONS
                    )
                ]
            ):
                self.__is_moving = False
                is_back_effect.set(None)

        return handler

    @xronos.reaction
    def on_rotate(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.rotate)
        joint_effects = [
            interface.add_effect(joint.target_position) for joint in self.__joints
        ]

        def handler() -> None:
            self.__is_moving = True
            for joint, pos in zip(joint_effects, self.ROTATED_POSITIONS):
                joint.set(pos)

        return handler

    @xronos.reaction
    def on_rotate_back(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.rotate_back)
        joint_effects = [
            interface.add_effect(joint.target_position) for joint in self.__joints
        ]

        def handler() -> None:
            self.__is_moving = True
            for joint, pos in zip(joint_effects, self.NEUTRAL_POSITIONS):
                joint.set(pos)

        return handler


def main() -> None:
    robot = webots.Robot()

    robot_name = sys.argv[1]
    arm_speed = float(sys.argv[2])

    env = xronos.Environment()

    env.enable_telemetry(application_name=robot_name)

    simulator = env.create_reactor("simulator", Simulator, robot)
    controller = env.create_reactor("controller", Controller)
    distance_sensor = env.create_reactor("distance_sensor", DistanceSensor, robot)
    hand = env.create_reactor("hand", Hand, robot)
    arm = env.create_reactor("arm", Arm, robot, arm_speed)

    env.connect(simulator.done_step, arm.step)
    env.connect(simulator.done_step, distance_sensor.step)
    env.connect(simulator.done_step, hand.step)
    env.connect(controller.hand_grasp, hand.grasp)
    env.connect(controller.hand_release, hand.release)
    env.connect(controller.arm_rotate, arm.rotate)
    env.connect(controller.arm_rotate_back, arm.rotate_back)
    env.connect(hand.is_released, controller.hand_is_released)
    env.connect(hand.is_grasped, controller.hand_is_grasped)
    env.connect(arm.is_rotated, controller.arm_is_rotated)
    env.connect(arm.is_back, controller.arm_is_back)
    env.connect(distance_sensor.out_current_distance, controller.distance)

    env.execute()

    robot.cleanup()


if __name__ == "__main__":
    main()

# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""This example serves as a general style guide for writing Reactors."""

import datetime
import threading
import time
from typing import Callable

import xronos


class MyReactor(xronos.Reactor):
    # Ports should be public attributes, as they are accessed by the enviornment
    # when making connections to other reactors.
    #
    # Name the port in a way that will make sense when making connections to and
    # from other reactors.
    #
    # Here, use the name `input_` instead of `input` to avoid a keyword collision
    # with Python -- or better, use a more descriptive name than `input`.
    input_ = xronos.InputPortDeclaration[float]()
    output = xronos.OutputPortDeclaration[float]()

    # Physical events may be public attributes if raised by external processes,
    # or private if they are raised by processes internal to this reactor.
    #
    # Use a descriptive name for what the event signals, and from what process
    # and under which conditions the event is raised. In this example, it signals
    # that a sensor value has been produced by an external process (such as an
    # interrupt) with a floating-pint value.
    _sensor = xronos.PhysicalEventDeclaration[float]()

    # Programmable timers should always be private -- they are intended for use for
    # messaging within a reactor only. If you need to communicate to another
    # reactor, use ports.
    #
    # Use a descriptive name for what an programmed event signals. In this
    # example, when the timer triggers, it signals an actuation with a
    # floating-point should occur.
    _actuate = xronos.ProgrammableTimerDeclaration[float]()

    # Periodic timer should in general be private.
    _periodic_timer = xronos.PeriodicTimerDeclaration()

    # reactor state variables
    _sensor_reads = 0

    # reactor constants
    MAX_SENSOR_READS = 3
    SENSOR_READING_DELAY = datetime.timedelta(milliseconds=50)

    def __init__(self) -> None:
        super().__init__()

        self._periodic_timer.period = datetime.timedelta(milliseconds=500)
        self._periodic_timer.offset = self._periodic_timer.period

        # threads should be private, started on a triggered event such as startup,
        # signaled using a state variable, and joined on shutdown.
        self._thread: threading.Thread = threading.Thread()

        # variables used to signal threads should end with `request`
        self._shutdown_request: bool = False

    # suffix methods intended to run in threads with `process`
    def read_sensor_process(self) -> None:
        while not self._shutdown_request:
            # simulate a blocking read by sleeping
            time.sleep(self.SENSOR_READING_DELAY.total_seconds())
            self._sensor.trigger(42)  # imagine this is a value from a sensor

    # reaction specifications begin with the event and have prefix `on`
    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the reaction to the startup event."""
        interface.add_trigger(self.startup)

        # use the name `handler()` for the reaction body
        def handler() -> None:
            self._thread = threading.Thread(target=self.read_sensor_process)
            self._thread.start()

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the reaction to the shutdown event."""
        interface.add_trigger(self.shutdown)

        # use the name `handler()` for the reaction body
        def handler() -> None:
            # signal threads to terminate
            self._shutdown_request = True
            # join (block) only on a shutdown reaction
            self._thread.join()

        return handler

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_trigger = interface.add_trigger(self.input_)
        output_effect = interface.add_effect(self.output)

        def handler() -> None:
            output_effect.set(input_trigger.get())

        return handler

    @xronos.reaction
    def on_sensor(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the reaction to the sensor action."""
        sensor_trigger = interface.add_trigger(self._sensor)
        shutown_effect = interface.add_effect(self.shutdown)

        def handler() -> None:
            print(f"Sensor read: {sensor_trigger.get()}")
            self._sensor_reads += 1
            if self._sensor_reads == self.MAX_SENSOR_READS:
                shutown_effect.trigger_shutdown()

        return handler

    @xronos.reaction
    def on_actuate(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        actuate_trigger = interface.add_trigger(self._actuate)

        def handler() -> None:
            print(f"Actuating with value {actuate_trigger.get()}")

        return handler

    @xronos.reaction
    def on_periodic_timer(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self._periodic_timer)

        # prefix effects with the port or action name and suffix with `effect`
        actuate_effect = interface.add_effect(self._actuate)

        def handler() -> None:
            actuate_effect.schedule(1.0)

        return handler


def test_style_guide() -> None:
    main(True)


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    env.create_reactor("MyReactor", MyReactor)
    env.execute()


if __name__ == "__main__":
    main()

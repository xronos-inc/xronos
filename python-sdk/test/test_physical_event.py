# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import threading
from time import sleep
from typing import Callable

import xronos

NUM_ITERATIONS = 5


class Sensor(xronos.Reactor):
    event = xronos.PhysicalEventDeclaration[int]()

    def __init__(self) -> None:
        super().__init__()
        self.__count = 0

    @xronos.reaction
    def on_event(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        event_trigger = interface.add_trigger(self.event)

        def handler() -> None:
            print(f"received {event_trigger.get()}")
            assert event_trigger.get() == self.__count
            self.__count += 1

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.shutdown)

        def handler() -> None:
            assert self.__count == NUM_ITERATIONS

        return handler


def run(env: xronos.Environment) -> None:
    sensor = env.create_reactor("sensor", Sensor)
    xronos_thread = threading.Thread(target=env.execute)
    xronos_thread.start()
    # Triggering the physial event only takes effect once the program is
    # started. Sleeping makes sure the program is started up before we trigger
    # for the first time.
    sleep(0.1)
    for i in range(0, NUM_ITERATIONS):
        sensor.event.trigger(i)
    xronos_thread.join()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=200), fast=fast)
    run(env)


def test_action() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=200))
    run(env)


if __name__ == "__main__":
    main()

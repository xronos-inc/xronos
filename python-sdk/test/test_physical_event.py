# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import threading
from collections.abc import Callable
from time import monotonic, sleep

import xronos

NUM_ITERATIONS = 5


class Sensor(xronos.Reactor):
    event = xronos.PhysicalEventDeclaration[int]()

    def __init__(self) -> None:
        super().__init__()
        self.__count = 0

    @xronos.reaction
    def on_event(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        event_trigger = ctx.add_trigger(self.event)

        def handler() -> None:
            print(f"received {event_trigger.get()}")
            assert event_trigger.get() == self.__count
            self.__count += 1

        return handler

    @xronos.reaction
    def on_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self.shutdown)

        def handler() -> None:
            assert self.__count == NUM_ITERATIONS

        return handler


def run(env: xronos.Environment) -> None:
    sensor = env.create_reactor("sensor", Sensor)
    xronos_thread = threading.Thread(target=env.execute)
    xronos_thread.start()
    # Triggering the physical event only takes effect once the program has
    # started. The first trigger retries until the program accepts it; a
    # dropped attempt reports NOT_STARTED and delivers nothing.
    deadline = monotonic() + 5.0
    while (status := sensor.event.trigger(0)) is xronos.TriggerStatus.NOT_STARTED:
        assert monotonic() < deadline, "the program did not start within 5 seconds"
        sleep(0.001)
    assert status is xronos.TriggerStatus.ACCEPTED
    # The program is now live, so the remaining triggers must be accepted
    # directly.
    for i in range(1, NUM_ITERATIONS):
        assert sensor.event.trigger(i) is xronos.TriggerStatus.ACCEPTED
    xronos_thread.join()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=200), fast=fast)
    run(env)


def test_action() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=200))
    run(env)


class Recorder(xronos.Reactor):
    event = xronos.PhysicalEventDeclaration[int]()

    def __init__(self) -> None:
        super().__init__()
        self.count = 0

    @xronos.reaction
    def on_event(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        event_trigger = ctx.add_trigger(self.event)

        def handler() -> None:
            _ = event_trigger.get()
            self.count += 1

        return handler


def test_status_outside_live_window() -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=10))
    recorder = env.create_reactor("recorder", Recorder)
    assert recorder.event.trigger(0) is xronos.TriggerStatus.NOT_STARTED
    env.execute()
    assert recorder.event.trigger(1) is xronos.TriggerStatus.STOPPED
    assert recorder.count == 0


def test_trigger_across_end_of_execution() -> None:
    """Hammer a physical event from a foreign thread while the program stops.

    STOPPED is the documented exit signal for sensor threads: once execute()
    has returned, every further attempt reports it, so the worker is
    guaranteed to observe it and exit. The worker only records; the
    assertions run after the join.
    """
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=200))
    recorder = env.create_reactor("recorder", Recorder)
    unexpected: list[object] = []
    observed_accepted = False

    def hammer() -> None:
        nonlocal observed_accepted
        deadline = monotonic() + 30.0
        while monotonic() < deadline:
            status = recorder.event.trigger(0)
            if status is xronos.TriggerStatus.STOPPED:
                return
            if status is xronos.TriggerStatus.ACCEPTED:
                observed_accepted = True
            elif status is not xronos.TriggerStatus.NOT_STARTED:
                unexpected.append(status)
        unexpected.append("the worker never observed STOPPED")

    worker = threading.Thread(target=hammer)
    worker.start()
    env.execute()
    worker.join()

    assert not unexpected
    # Without an accepted fire the test would not have raced a live delivery
    # against the end of execution at all.
    assert observed_accepted
    # The environment is still alive, so a late attempt keeps reporting
    # STOPPED instead of reaching into the finished program.
    assert recorder.event.trigger(1) is xronos.TriggerStatus.STOPPED


if __name__ == "__main__":
    main()

# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos

DELAY0 = datetime.timedelta(milliseconds=100)
DELAY1 = datetime.timedelta(milliseconds=200)


def assert_times_equal(time1: datetime.timedelta, time2: datetime.timedelta) -> None:
    assert time1 == time2, f"{time1} != {time2}"
    print(f"OK: {time1} == {time2}")


class Test(xronos.Reactor):
    __test__ = False  # tell pytest to not collect tests from this class

    delay0 = xronos.InputPortDeclaration[None]()
    nodelay = xronos.InputPortDeclaration[None]()
    delay1 = xronos.InputPortDeclaration[None]()

    out = xronos.OutputPortDeclaration[None]()

    NUM_REACTIONS_BEFORE_SHUTDOWN = 4

    def __init__(self) -> None:
        super().__init__()
        self._reactions_executed = 0

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.startup)
        out = interface.add_effect(self.out)

        def handler() -> None:
            out.set(None)
            self._reactions_executed += 1

        return handler

    @xronos.reaction
    def on_delay0(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.delay0)

        def handler() -> None:
            assert_times_equal(self.get_time_since_startup(), DELAY0)
            self._reactions_executed += 1

        return handler

    @xronos.reaction
    def on_nodelay(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.nodelay)

        def handler() -> None:
            assert_times_equal(
                self.get_time_since_startup(), datetime.timedelta(milliseconds=0)
            )
            self._reactions_executed += 1

        return handler

    @xronos.reaction
    def on_delay1(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.delay1)

        def handler() -> None:
            assert_times_equal(self.get_time_since_startup(), DELAY1)
            self._reactions_executed += 1

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.shutdown)

        def handler() -> None:
            assert self._reactions_executed == self.NUM_REACTIONS_BEFORE_SHUTDOWN
            print(f"{self.fqn} passed all tests")

        return handler


def run(env: xronos.Environment) -> None:
    test = env.create_reactor("test", Test)
    env.connect(test.out, test.delay0, delay=DELAY0)
    env.connect(test.out, test.nodelay)
    env.connect(test.out, test.delay1, delay=DELAY1)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=DELAY1, fast=fast)
    run(env)


def test_delay_and_nodelay() -> None:
    env = xronos.Environment(fast=True, timeout=DELAY1)
    run(env)


if __name__ == "__main__":
    main()

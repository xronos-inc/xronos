# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos


class GetTime(xronos.Reactor):
    timer = xronos.PeriodicTimerDeclaration(period=datetime.timedelta(milliseconds=5))

    def __init__(self) -> None:
        super().__init__()
        self.__startup_time: datetime.datetime | None = None
        self.__counter = 0

    @xronos.reaction
    def on_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        timer_trigger = interface.add_trigger(self.timer)
        startup_trigger = interface.add_trigger(self.startup)

        def handler() -> None:
            print("==================================")
            print(f"time: {self.get_time()}")
            print(f"lag: {self.get_lag()}")
            print(f"time since startup: {self.get_time_since_startup()}")

            if startup_trigger.is_present():
                assert timer_trigger.is_present()
                assert self.__startup_time is None
                assert self.get_time_since_startup() == datetime.timedelta(0)
                self.__startup_time = self.get_time()

            assert self.__startup_time
            assert self.get_time_since_startup().microseconds % 5000 == 0
            assert self.get_time_since_startup().microseconds / 5000 == self.__counter
            assert (
                self.get_time() - self.__startup_time
            ) == self.get_time_since_startup()

            self.__counter += 1

        return handler


def run(env: xronos.Environment) -> None:
    env.create_reactor("get_time", GetTime)
    env.execute()


def main() -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=20))
    run(env)


def test_get_time() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=20))
    run(env)


if __name__ == "__main__":
    main()

# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos


class Timed(xronos.Reactor):
    _timer = xronos.PeriodicTimerDeclaration()

    def __init__(self, period: datetime.timedelta) -> None:
        super().__init__()
        self._timer.period = period

    @xronos.reaction
    def on_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self._timer)
        return lambda: print(f"{self._timer.fqn} triggered")


def run(env: xronos.Environment) -> None:
    env.create_reactor("timed_1s", Timed, period=datetime.timedelta(seconds=1))
    env.create_reactor("timed_2s", Timed, period=datetime.timedelta(seconds=2))
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    run(env)


def test_timer() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    run(env)


if __name__ == "__main__":
    main()

# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import pytest

import xronos

FIRST_EXCEPTION_ITERATION = 3
SECOND_EXCEPTION_ITERATION = 5


class Timed(xronos.Reactor):
    _timer = xronos.PeriodicTimerDeclaration(period=datetime.timedelta(seconds=1))

    def __init__(self) -> None:
        super().__init__()
        self.count = 0

    @xronos.reaction
    def on_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self._timer)

        def handler() -> None:
            print(f"{self._timer.fqn} triggered")
            if self.count == FIRST_EXCEPTION_ITERATION:
                raise Exception(
                    f"Exception raised at count {FIRST_EXCEPTION_ITERATION},"
                    f" as expected"
                )
            self.count += 1
            if self.count == SECOND_EXCEPTION_ITERATION:
                raise Exception("This code should never be reached")

        return handler


def run(env: xronos.Environment) -> None:
    env.create_reactor("timed", Timed)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    run(env)


def test_exceptions() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    with pytest.raises(Exception, match=r"Exception raised at count 3, as expected"):
        run(env)


if __name__ == "__main__":
    main()

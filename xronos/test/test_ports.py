# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos


class Counter(xronos.Reactor):
    output = xronos.OutputPortDeclaration[int]()
    _timer = xronos.PeriodicTimerDeclaration()

    def __init__(
        self, period: datetime.timedelta, initial_count: int = 0, increment: int = 1
    ) -> None:
        super().__init__()

        # state
        self._count = initial_count
        self._increment = increment

        # reactor elements
        self._timer.period = period

    @xronos.reaction
    def on_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self._timer)
        output = interface.add_effect(self.output)

        def handler() -> None:
            self._count += self._increment
            output.set(self._count)
            print(f"{self.fqn} sent new count {self._count}")

        return handler


class Printer(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)
        return lambda: print(f"{self.fqn} received {input_.get()}")


def run(env: xronos.Environment) -> None:
    counter = env.create_reactor(
        "counter",
        Counter,
        period=datetime.timedelta(milliseconds=500),
        initial_count=42,
    )
    printer = env.create_reactor("printer", Printer)
    env.connect(counter.output, printer.input_)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    run(env)


def test_ports() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    run(env)


if __name__ == "__main__":
    main()

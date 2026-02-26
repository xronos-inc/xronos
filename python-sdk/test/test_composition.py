# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos

"""
This test connects a simple counting source to a receiver that checks
against its own count.
"""


class Source(xronos.Reactor):
    output_ = xronos.OutputPortDeclaration[int]()
    timer = xronos.PeriodicTimerDeclaration()

    def __init__(self, period: datetime.timedelta) -> None:
        super().__init__()
        self.timer.period = period
        self.count = 0

    @xronos.reaction
    def emit_count(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.timer)
        output_ = interface.add_effect(self.output_)

        def handler() -> None:
            self.count += 1
            output_.set(self.count)

        return handler


class Receiver(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()

    def __init__(self) -> None:
        super().__init__()
        self.count = 0

    @xronos.reaction
    def check_expected_count(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)

        def handler() -> None:
            self.count += 1
            assert self.count == input_.get()

        return handler


def run(env: xronos.Environment) -> None:
    receiver = env.create_reactor("receiver", Receiver)
    source = env.create_reactor("source", Source, datetime.timedelta(milliseconds=400))
    env.connect(source.output_, receiver.input_)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=3), fast=fast)
    run(env)


def test_composition() -> None:
    main(fast=True)


if __name__ == "__main__":
    main()

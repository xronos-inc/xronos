# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos

from .test_ports import Counter


class Printer(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)

        def handler() -> None:
            time = self.get_time_since_startup()
            print(f"{self.fqn} received {input_.get()} at {time}")
            if time < datetime.timedelta(milliseconds=100):
                raise Exception(
                    f"Received input at time {time} which is before the expected time"
                )

        return handler


def run(env: xronos.Environment) -> None:
    counter = env.create_reactor(
        "counter", Counter, period=datetime.timedelta(milliseconds=50), initial_count=0
    )
    printer = env.create_reactor("printer", Printer)
    env.connect(
        counter.output, printer.input_, delay=datetime.timedelta(milliseconds=100)
    )
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=250), fast=fast)
    run(env)


def test_ports() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=250))
    run(env)


if __name__ == "__main__":
    main()

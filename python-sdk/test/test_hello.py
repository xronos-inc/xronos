# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Hello(xronos.Reactor):
    @xronos.reaction
    def hello(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        return lambda: print(f"{self.fqn} says hello!")

    @xronos.reaction
    def goodbye(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)
        return lambda: print(f"{self.fqn} says goodbye!")


def run(env: xronos.Environment) -> None:
    env.create_reactor("hello", Hello)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast)
    run(env)


def test_hello() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

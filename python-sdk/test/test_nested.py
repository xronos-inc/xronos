# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos

from .test_hello import Hello


class Nested(xronos.Reactor):
    def __init__(self) -> None:
        super().__init__()
        self._hello = self.create_reactor("hello_chind", Hello)

    @xronos.reaction
    def hello(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        return lambda: print(f"{self.fqn} says hello!")

    @xronos.reaction
    def goodbye(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)
        return lambda: print(f"{self.fqn} says goodbye!")


def run(env: xronos.Environment) -> None:
    env.create_reactor("nested", Nested)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_nested() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

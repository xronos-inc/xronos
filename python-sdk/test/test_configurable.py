# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Configurable(xronos.Reactor):
    def __init__(self, at_startup: bool) -> None:
        super().__init__()
        self._at_startup = at_startup

    @xronos.reaction
    def hello(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        if self._at_startup:
            interface.add_trigger(self.startup)
            name = self.startup.name
        else:
            interface.add_trigger(self.shutdown)
            name = self.shutdown.name

        return lambda: print(f"{self.fqn} says 'Hello!' triggered by {name}")


def run(env: xronos.Environment) -> None:
    env.create_reactor("config_true", Configurable, at_startup=True)
    env.create_reactor("config_false", Configurable, at_startup=False)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_configurable() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

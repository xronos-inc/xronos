# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos

from .test_hello import Hello


class Inherited(Hello):
    @xronos.reaction
    def extra(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        startup = interface.add_trigger(self.startup)
        shutdown = interface.add_trigger(self.shutdown)

        def handler() -> None:
            if startup.is_present():
                print("extra triggered at startup")
            elif shutdown.is_present():
                print("extra triggered at shutdown")

        return handler


def run(env: xronos.Environment) -> None:
    env.create_reactor("inherited", Inherited)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_inherited() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

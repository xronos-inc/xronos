# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Variants(xronos.Reactor):
    @xronos.reaction
    def variant_t(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        startup = interface.add_trigger(self.startup)
        shutdown = interface.add_trigger(self.shutdown)

        def handler() -> None:
            if startup.is_present():
                print(f"{self.fqn}: variant_1 triggered at startup")
            elif shutdown.is_present():
                print(f"{self.fqn}: variant_1 triggered at shutdown")

        return handler

    @xronos.reaction
    class variant_2:
        def __init__(
            self, ctx: "Variants", interface: xronos.ReactionInterface
        ) -> None:
            self._startup = interface.add_trigger(ctx.startup)
            self._shutdown = interface.add_trigger(ctx.shutdown)
            self._ctx = ctx

        def __call__(self) -> None:
            if self._startup.is_present():
                print(f"{self._ctx.fqn}: variant_2 triggered at startup")
            elif self._shutdown.is_present():
                print(f"{self._ctx.fqn}: variant_2 triggered at shutdown")

    @xronos.reaction
    def variant_3(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        startup = interface.add_trigger(self.startup)
        shutdown = interface.add_trigger(self.shutdown)

        return lambda: type(self)._variant_3_handler(self, startup, shutdown)

    def _variant_3_handler(
        self, startup: xronos.Trigger[None], shutdown: xronos.Trigger[None]
    ) -> None:
        if startup.is_present():
            print(f"{self.fqn}: variant_3 triggered at startup")
        elif shutdown.is_present():
            print(f"{self.fqn}: variant_3 triggered at shutdown")


def run(env: xronos.Environment) -> None:
    env.create_reactor("variants", Variants)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_syntax_variants() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

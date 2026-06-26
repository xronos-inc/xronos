# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Hello(xronos.Reactor):
    hello = xronos.OutputPortDeclaration[str]()

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self.startup)
        hello_effect = ctx.add_effect(self.hello)

        return lambda: hello_effect.set("Hello, World!")


class Printer(xronos.Reactor):
    hello = xronos.InputPortDeclaration[str]()

    @xronos.reaction
    def on_hello(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        hello_trigger = ctx.add_trigger(self.hello)

        return lambda: print(hello_trigger.get())


def main() -> None:
    env = xronos.Environment()
    hello = env.create_reactor("hello", Hello)
    printer = env.create_reactor("printer", Printer)
    env.connect(hello.hello, printer.hello)
    env.execute()


if __name__ == "__main__":
    main()

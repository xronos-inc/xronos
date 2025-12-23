# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Hello(xronos.Reactor):
    hello = xronos.OutputPortDeclaration[str]()

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.startup)
        hello_effect = interface.add_effect(self.hello)

        return lambda: hello_effect.set("Hello, World!")


class Printer(xronos.Reactor):
    hello = xronos.InputPortDeclaration[str]()

    @xronos.reaction
    def on_hello(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        hello_trigger = interface.add_trigger(self.hello)

        return lambda: print(hello_trigger.get())


def main() -> None:
    env = xronos.Environment()
    hello = env.create_reactor("hello", Hello)
    printer = env.create_reactor("printer", Printer)
    env.connect(hello.hello, printer.hello)
    env.execute()


if __name__ == "__main__":
    main()

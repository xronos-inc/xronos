# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Receiver(xronos.Reactor):
    input = xronos.InputPortDeclaration[int]()

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input)

        def handler() -> None:
            assert input_.get() == Gain.SCALE

        return handler


class Source(xronos.Reactor):
    output = xronos.OutputPortDeclaration[int]()

    @xronos.reaction
    def on_start(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        output_ = interface.add_effect(self.output)

        def handler() -> None:
            output_.set(1)

        return handler


class Gain(xronos.Reactor):
    SCALE = 2

    input = xronos.InputPortDeclaration[int]()
    output = xronos.OutputPortDeclaration[int]()

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input)
        output_ = interface.add_effect(self.output)

        def handler() -> None:
            output_.set(input_.get() * Gain.SCALE)

        return handler


class GainContainer(xronos.Reactor):
    input = xronos.InputPortDeclaration[int]()
    output1 = xronos.OutputPortDeclaration[int]()
    output2 = xronos.OutputPortDeclaration[int]()

    def __init__(self) -> None:
        super().__init__()

        self.gain = self.create_reactor("gain", Gain)

        self.connect(self.input, self.gain.input)
        self.connect(self.gain.output, self.output1)
        self.connect(self.gain.output, self.output2)


def run(env: xronos.Environment) -> None:
    source = env.create_reactor("source", Source)
    container = env.create_reactor("container", GainContainer)
    receiver1 = env.create_reactor("r1", Receiver)
    receiver2 = env.create_reactor("r2", Receiver)

    env.connect(source.output, container.input)
    env.connect(container.output1, receiver1.input)
    env.connect(container.output1, receiver2.input)
    env.execute()


def main() -> None:
    env = xronos.Environment()
    run(env)


def test_hierarchy() -> None:
    main()


if __name__ == "__main__":
    main()

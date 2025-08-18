# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import pytest

import xronos


class ZeroDelayInOut(xronos.Reactor):
    input = xronos.InputPortDeclaration[int]()
    output = xronos.OutputPortDeclaration[int]()

    @xronos.reaction
    def passthrough(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.input)
        interface.add_effect(self.output)

        def ret() -> None:
            raise Exception("This should be unreachable")

        return ret


def run(env: xronos.Environment) -> None:
    zdio0 = env.create_reactor("zdio0", ZeroDelayInOut)
    zdio1 = env.create_reactor("zdio1", ZeroDelayInOut)
    env.connect(zdio0.output, zdio1.input)
    env.connect(zdio1.output, zdio0.input)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_exception_causality_cycle() -> None:
    env = xronos.Environment(fast=True)

    with pytest.raises(
        xronos.ValidationError,
        match=r"There is a loop in the dependency graph. Graph was written to "
        "/tmp/reactor_dependency_graph.dot",
    ):
        run(env)


if __name__ == "__main__":
    main()

# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import re
from typing import Generic, TypeVar

import pytest  # pyright: ignore

import xronos
from xronos.lib._simple_source import StartupSource

T = TypeVar("T")


class StartupAndShutdownSource(xronos.Reactor, Generic[T]):
    """A reactor that produces a single output at startup and shutdown."""

    output = xronos.OutputPortDeclaration[T]()

    def __init__(self, startup_value: T, shutdown_value: T) -> None:
        super().__init__()
        self._startup_reactor1 = self.create_reactor(
            "StartupSource1", StartupSource[T], value=startup_value
        )
        self._startup_reactor2 = self.create_reactor(
            "StartupSource2", StartupSource[T], value=startup_value
        )

        self.connect(self._startup_reactor1.output, self.output)

        with pytest.raises(  # pyright: ignore
            xronos.ValidationError,
            match=re.escape(
                "Cannot connect port invalid.StartupSource2.output to port "
                "invalid.output because it already has an inbound connection from "
                "port invalid.StartupSource1.output. Each port may have at most one "
                "inbound connection."
            ),
        ):
            self.connect(self._startup_reactor2.output, self.output)


def run(env: xronos.Environment) -> None:
    env.create_reactor(
        "invalid", StartupAndShutdownSource, startup_value=1, shutdown_value=2
    )


def main(fast: bool = False) -> xronos.Environment:
    env = xronos.Environment(fast=fast)
    run(env)
    return env


def test_multiple_sources_same_port() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Generic, TypeVar

import pytest  # pyright: ignore

import xronos
from xronos.lib._simple_source import StartupSource
from xronos.lib.test.test_simple_source import ShutdownSource

T = TypeVar("T")


class StartupAndShutdownSource(xronos.Reactor, Generic[T]):
    """A reactor that produces a single output at startup and shutdown."""

    output = xronos.OutputPortDeclaration[T]()

    def __init__(self, startup_value: T, shutdown_value: T) -> None:
        super().__init__()
        self._startup_reactor = self.create_reactor(
            "StartupSource", StartupSource[T], value=startup_value
        )
        self._shutdown_reactor = self.create_reactor(
            "ShutdownSource", ShutdownSource[T], value=shutdown_value
        )

        self.connect(self._startup_reactor.output, self.output)

        with pytest.raises(  # pyright: ignore
            xronos.ValidationError,
            match=r"multiple\ sources\ \(both\ invalid.(Startup|Shutdown)Source.output"
            r"\ and\ invalid.(Startup|Shutdown)Source.output\)\ connected\ to\ port\ "
            r"invalid.output",
        ):
            self.connect(self._shutdown_reactor.output, self.output)


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

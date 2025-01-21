# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import io
from collections.abc import Generator

import pytest
from pytest import MonkeyPatch

import xronos
from xronos.lib import ConsoleInput, ExternalInput
from xronos.lib.test.util import AssertList


def test_external_input_list() -> None:
    """Test ExternalInput with a generator that produces a list of values."""
    env = xronos.Environment()

    def read_input() -> Generator[int]:
        yield from range(100)

    testInput = list(range(100))
    r = env.create_reactor(
        "ExternalInputList",
        ExternalInput[int],
        read_input=read_input(),
    )
    asserter = env.create_reactor("AssertList", AssertList[int], expected=testInput)
    env.connect(r.output, asserter.input_)
    env.execute()


def test_external_input_list_with_backpressure() -> None:
    """Test ExternalInput input with backpressure."""
    env = xronos.Environment()

    def read_input() -> Generator[int]:
        yield from range(100)

    testInput = list(range(100))
    r = env.create_reactor(
        "ExternalInputList",
        ExternalInput[int],
        read_input=read_input(),
    )
    asserter = env.create_reactor(
        "AssertList",
        AssertList[int],
        expected=testInput,
    )
    env.connect(r.output, asserter.input_)
    env.execute()


def test_external_input_teardown() -> None:
    """Test that close method of the ExternalInput generator is called."""
    env = xronos.Environment()

    teardown_executed = False

    def read_input() -> Generator[int]:
        # Outputs the numbers from 0 to 99 until GeneratorExit is raised.
        try:
            while True:
                yield from range(100)
        except GeneratorExit:
            # This exception is raised when the generator is closed.
            pass

        nonlocal teardown_executed
        teardown_executed = True

    testInput = list(range(100))
    r = env.create_reactor(
        "ExternalInputList",
        ExternalInput[int],
        read_input=read_input(),
    )
    asserter = env.create_reactor(
        "AssertList", AssertList[int], expected=testInput, debug=True
    )
    env.connect(r.output, asserter.input_)
    env.execute()

    assert teardown_executed


def test_external_input_exception() -> None:
    """Test that when an exception is raised, it is propagated to the runtime."""
    with pytest.raises(ValueError):

        def read_input() -> Generator[int]:
            yield 1
            raise ValueError("Test exception")

        env = xronos.Environment()
        env.create_reactor(
            "ExternalInputList",
            ExternalInput[int],
            read_input=read_input(),
        )
        env.execute()


def test_stdin_external_input(monkeypatch: MonkeyPatch) -> None:
    """Test the ConsoleInput reactor by monkeypatching sys.stdin."""
    testInput = list(range(100))
    monkeypatch.setattr("sys.stdin", io.StringIO("\n".join(map(str, testInput)) + "\n"))

    def parser(x: str) -> int:
        return int(x)

    env = xronos.Environment()
    r = env.create_reactor(
        "ConsoleInput",
        ConsoleInput[int],
        parser=parser,
    )
    asserter = env.create_reactor(
        "ExternalInputAssert", AssertList[int], expected=testInput
    )
    env.connect(r.output, asserter.input_)
    env.execute()

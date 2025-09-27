# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import math
import sys
from typing import Callable, Generic, TypeVar, cast

import pytest

# 'override' added to typing in 3.12
from typing_extensions import override

import xronos
from xronos.lib import (
    ConstSource,
    OutputDiscardedWarning,
    RampSource,
    StartupSource,
    SuccessorSource,
    TimerSource,
)
from xronos.lib._abstract_source import AbstractTimerSource

T = TypeVar("T")


class PrintAndAssert(xronos.Reactor, Generic[T]):
    """Print and assert input values.

    Args:
        assert_value: A callable that returns the value to assert for each input.
            This function is called for every input that is received. Use a lambda
            to capture stateful behavior.
    """

    result = xronos.InputPortDeclaration[T]()

    def __init__(self, assert_value: Callable[[], T]):
        super().__init__()
        self._assert_value: Callable[[], T] = assert_value
        self._assert_count: int = 0

    @xronos.reaction
    def _on_print(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_trigger = interface.add_trigger(self.result)

        def body() -> None:
            check_value = self._assert_value()
            self._assert_count += 1
            print(
                f"<PrintAndAssert> #{self._assert_count}"
                + f" expects {check_value} and received {input_trigger.get()}"
            )
            assert check_value == input_trigger.get()

        return body

    def count(self) -> int:
        return self._assert_count


class SimultaneousSource(AbstractTimerSource[T]):
    """A reactor that produces simultaneous outputs at startup."""

    def __init__(self, startup_value: T, timer_value: T) -> None:
        super().__init__(
            period=datetime.timedelta(seconds=5), offset=datetime.timedelta(seconds=0)
        )
        self.__startup_value: T = startup_value
        self.__timer_value: T = timer_value

    @override
    def _startup_handler(self) -> T:
        return self.__startup_value

    @override
    def _timer_handler(self) -> T:
        return self.__timer_value


def test_startup_source() -> None:
    """Test the AbstractSource with only a startup trigger."""
    env = xronos.Environment()
    value = "hello"
    printer = env.create_reactor(
        "Startup Printer", PrintAndAssert[str], assert_value=lambda: value
    )
    env.connect(
        env.create_reactor("Startup Source", StartupSource, value=value).output,
        printer.result,
    )
    env.execute()
    assert printer.count() == 1


@pytest.mark.skipif(
    sys.version_info > (3, 13) and not sys._is_gil_enabled(),
    reason="Catching warnings currently does not work as expected in Python 3.14t",
)
def test_simultaneous_events() -> None:
    """Test the AbstractSource with simultaneous events."""
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=1))
    values = ["startup", "timer"]

    printer = env.create_reactor(
        "Simultaneous Printer",
        PrintAndAssert[str],
        assert_value=lambda: values[-1],  # last value wins
    )
    source = env.create_reactor(
        "Simultaneous Source",
        SimultaneousSource,
        startup_value=values[0],
        timer_value=values[1],
    )
    env.connect(
        source.output,
        printer.result,
    )

    with pytest.warns(OutputDiscardedWarning) as warning_record:
        env.execute()
        assert len(warning_record) == 1
        assert (
            cast(OutputDiscardedWarning, warning_record[0].message).value == values[0]
        )


def test_timed_source() -> None:
    """Test a TimedSource with a periodic trigger."""
    duration = datetime.timedelta(seconds=5)
    period = datetime.timedelta(seconds=1)
    env = xronos.Environment(timeout=duration, fast=True)
    printer = env.create_reactor(
        "PrintAndAssert", PrintAndAssert[int], assert_value=lambda: 42
    )
    timer = env.create_reactor(name="Timer", reactor_class=TimerSource, period=period)
    source = env.create_reactor(
        name="ConstSource",
        reactor_class=ConstSource[int],
        value=42,
    )
    env.connect(timer.output, source.trigger)
    env.connect(source.output, printer.result)
    env.execute()
    assert printer.count() == math.floor(duration / period) + 1


def test_successor_source() -> None:
    """Test a SuccessorSource with a periodic trigger."""
    duration = datetime.timedelta(seconds=5)
    period = datetime.timedelta(seconds=1)
    env = xronos.Environment(timeout=duration, fast=True)

    def fib(n: int) -> int:
        if n <= 1:
            return max(0, n)
        else:
            return fib(n - 1) + fib(n - 2)

    printer = env.create_reactor(
        "PrintAndAssert",
        PrintAndAssert[tuple[int, int]],
        assert_value=lambda: (fib(printer.count()), fib(printer.count() + 1)),
    )
    timer = env.create_reactor("TimerSource", TimerSource, period=period)
    source = env.create_reactor(
        "SuccessorSource",
        SuccessorSource[tuple[int, int]],
        initial_value=(0, 1),
        successor=lambda x: (x[1], x[0] + x[1]),
    )
    env.connect(timer.output, source.trigger)
    env.connect(source.output, printer.result)
    env.execute()
    assert printer.count() == math.floor(duration / period) + 1


def test_ramp_source() -> None:
    """Test a RampSource with a periodic trigger."""
    duration = datetime.timedelta(seconds=5)
    period = datetime.timedelta(seconds=1)
    env = xronos.Environment(timeout=duration, fast=True)
    printer = env.create_reactor(
        "PrintAndAssert", PrintAndAssert[int], assert_value=lambda: printer.count()
    )
    timer = env.create_reactor("TimerSource", TimerSource, period=period)
    ramp = env.create_reactor(
        "RampSource",
        RampSource[int],
    )
    env.connect(timer.output, ramp.trigger)
    env.connect(ramp.output, printer.result)
    env.execute()
    assert printer.count() == math.floor(duration / period) + 1

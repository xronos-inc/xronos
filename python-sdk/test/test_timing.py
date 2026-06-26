# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import pytest

import xronos


class GetTime(xronos.Reactor):
    timer = xronos.PeriodicTimerDeclaration(period=datetime.timedelta(milliseconds=5))

    def __init__(self) -> None:
        super().__init__()
        self.__startup_time: datetime.datetime | None = None
        self.__counter = 0

    @xronos.reaction
    def on_timer(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        timer_trigger = ctx.add_trigger(self.timer)
        startup_trigger = ctx.add_trigger(self.startup)

        def handler() -> None:
            print("==================================")
            print(f"time: {ctx.current_time}")
            print(f"lag: {ctx.lag}")
            print(f"time since startup: {ctx.elapsed_time}")

            if startup_trigger.is_present():
                assert timer_trigger.is_present()
                assert self.__startup_time is None
                assert ctx.elapsed_time == datetime.timedelta(0)
                self.__startup_time = ctx.current_time

            assert self.__startup_time
            assert ctx.elapsed_time.microseconds % 5000 == 0
            assert ctx.elapsed_time.microseconds / 5000 == self.__counter
            assert (ctx.current_time - self.__startup_time) == ctx.elapsed_time

            self.__counter += 1

        return handler


def run(env: xronos.Environment) -> None:
    env.create_reactor("get_time", GetTime)
    env.execute()


def main() -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=20))
    run(env)


def test_get_time() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=20))
    run(env)


class TimingDuringDeclaration(xronos.Reactor):
    """Reads the timing API while the reaction is being declared.

    Before execution begins the runtime clock is not yet defined, so the
    reaction-scoped timing API must return the defined placeholders (the epoch
    and zero) rather than crashing or returning garbage.
    """

    def __init__(self) -> None:
        super().__init__()
        self.declaration_time: datetime.datetime | None = None
        self.declaration_lag: datetime.timedelta | None = None
        self.declaration_time_since_startup: datetime.timedelta | None = None
        self.reaction_executed = False

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        # Timing is accessible during declaration, where it returns placeholders.
        self.declaration_time = ctx.current_time
        self.declaration_lag = ctx.lag
        self.declaration_time_since_startup = ctx.elapsed_time

        def handler() -> None:
            self.reaction_executed = True

        return handler


def test_timing_defaults_during_declaration() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=1))
    reactor = env.create_reactor("init_defaults", TimingDuringDeclaration)
    env.execute()

    assert reactor.reaction_executed
    # The defined placeholders: epoch for current_time, zero for the durations. In
    # particular lag must be zero (not the historical multi-decade bug).
    assert reactor.declaration_time == datetime.datetime.fromtimestamp(0)
    assert reactor.declaration_lag == datetime.timedelta(0)
    assert reactor.declaration_time_since_startup == datetime.timedelta(0)


def test_reactor_timing_is_deprecated() -> None:
    """The reactor-level timing API is deprecated but still functional."""
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=1))
    reactor = env.create_reactor("deprecated", GetTime)

    with pytest.warns(DeprecationWarning):
        reactor.get_time()  # pyright: ignore[reportDeprecated]
    with pytest.warns(DeprecationWarning):
        reactor.get_lag()  # pyright: ignore[reportDeprecated]
    with pytest.warns(DeprecationWarning):
        reactor.get_time_since_startup()  # pyright: ignore[reportDeprecated]


if __name__ == "__main__":
    main()

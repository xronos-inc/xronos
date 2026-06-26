# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import time
from typing import Callable

import pytest

import xronos

_EXPECTED_TIMER_COUNT = 6


class WithDeadlines(xronos.Reactor):
    """Declares deadlines and checks the getters while the handlers run."""

    timer = xronos.PeriodicTimerDeclaration(period=datetime.timedelta(seconds=1))

    def __init__(self) -> None:
        super().__init__()
        self.timer_count = 0
        self.startup_executed = False
        self.shutdown_executed = False

    @xronos.reaction_with_deadline(deadline=datetime.timedelta(milliseconds=100))
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        def handler() -> None:
            assert not self.startup_executed
            assert ctx.deadline is not None
            assert ctx.deadline - ctx.current_time == datetime.timedelta(
                milliseconds=100
            )
            assert ctx.slack > datetime.timedelta(0)
            assert ctx.is_before_deadline
            self.startup_executed = True

        return handler

    @xronos.reaction_with_deadline(deadline=datetime.timedelta(milliseconds=50))
    def on_timer(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.timer)

        def handler() -> None:
            assert ctx.deadline is not None
            assert ctx.deadline - ctx.current_time == datetime.timedelta(
                milliseconds=50
            )
            assert ctx.slack > datetime.timedelta(0)
            assert ctx.is_before_deadline
            self.timer_count += 1

        return handler

    @xronos.reaction_with_deadline(deadline=datetime.timedelta(milliseconds=200))
    def on_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.shutdown)

        def handler() -> None:
            assert not self.shutdown_executed
            assert ctx.deadline is not None
            assert ctx.deadline - ctx.current_time == datetime.timedelta(
                milliseconds=200
            )
            assert ctx.slack > datetime.timedelta(0)
            assert ctx.is_before_deadline
            self.shutdown_executed = True

        return handler


def test_deadline_getters() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    reactor = env.create_reactor("with_deadlines", WithDeadlines)
    env.execute()

    assert reactor.startup_executed
    assert reactor.shutdown_executed
    # Timer events at 0s, 1s, 2s, 3s, 4s, 5s.
    assert reactor.timer_count == _EXPECTED_TIMER_COUNT


class WithoutDeadlines(xronos.Reactor):
    """Checks that the getters are well-defined when no deadline is declared."""

    timer = xronos.PeriodicTimerDeclaration(period=datetime.timedelta(seconds=1))

    def __init__(self) -> None:
        super().__init__()
        self.timer_count = 0
        self.startup_executed = False
        self.shutdown_executed = False

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        def handler() -> None:
            assert ctx.deadline is None
            assert ctx.slack == datetime.timedelta.max
            assert ctx.is_before_deadline
            self.startup_executed = True

        return handler

    @xronos.reaction
    def on_timer(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.timer)

        def handler() -> None:
            assert ctx.deadline is None
            assert ctx.slack == datetime.timedelta.max
            assert ctx.is_before_deadline
            self.timer_count += 1

        return handler

    @xronos.reaction
    def on_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.shutdown)

        def handler() -> None:
            assert ctx.deadline is None
            assert ctx.slack == datetime.timedelta.max
            assert ctx.is_before_deadline
            self.shutdown_executed = True

        return handler


def test_deadline_getters_no_deadlines() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    reactor = env.create_reactor("without_deadlines", WithoutDeadlines)
    env.execute()

    assert reactor.startup_executed
    assert reactor.shutdown_executed
    assert reactor.timer_count == _EXPECTED_TIMER_COUNT


class ViolatedDeadline(xronos.Reactor):
    """Violates a zero deadline by sleeping inside the handler."""

    def __init__(self) -> None:
        super().__init__()
        self.executed = False

    @xronos.reaction_with_deadline(deadline=datetime.timedelta(0))
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        def handler() -> None:
            # The deadline is equal to the triggering event's timestamp. Once
            # any wall-clock time has passed, the deadline is violated.
            time.sleep(0.01)
            assert ctx.deadline is not None
            assert ctx.slack < datetime.timedelta(0)
            assert not ctx.is_before_deadline
            self.executed = True

        return handler


def test_violated_deadline() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=1))
    reactor = env.create_reactor("violated", ViolatedDeadline)
    env.execute()

    assert reactor.executed


class DeadlineDuringDeclaration(xronos.Reactor):
    """Reads the deadline getters while the reaction is being declared.

    Before execution begins the runtime clock is not yet defined, so the
    deadline getters must return their placeholders (``None`` / effectively
    infinite slack / ``True``) rather than crashing.
    """

    def __init__(self) -> None:
        super().__init__()
        self.declaration_deadline: datetime.datetime | None = None
        self.declaration_slack: datetime.timedelta | None = None
        self.declaration_is_before: bool | None = None
        self.reaction_executed = False

    @xronos.reaction_with_deadline(deadline=datetime.timedelta(milliseconds=100))
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        # The getters are callable during declaration, where they return
        # placeholders even though a deadline is declared.
        self.declaration_deadline = ctx.deadline
        self.declaration_slack = ctx.slack
        self.declaration_is_before = ctx.is_before_deadline

        def handler() -> None:
            self.reaction_executed = True

        return handler


def test_deadline_defaults_during_declaration() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=1))
    reactor = env.create_reactor("declaration", DeadlineDuringDeclaration)
    env.execute()

    assert reactor.reaction_executed
    assert reactor.declaration_deadline is None
    assert reactor.declaration_slack == datetime.timedelta.max
    assert reactor.declaration_is_before is True


def test_negative_deadline_rejected() -> None:
    # The decorator validates eagerly, so the error is raised as soon as the
    # negative deadline is passed.
    with pytest.raises(ValueError):
        xronos.reaction_with_deadline(deadline=datetime.timedelta(milliseconds=-1))


def test_decorator_forms_are_accepted() -> None:
    """The bare and deadline decorator forms both produce reactions."""

    class Reactor(xronos.Reactor):
        def __init__(self) -> None:
            super().__init__()
            self.bare_ran = False
            self.with_deadline_ran = False

        @xronos.reaction
        def bare(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
            ctx.add_trigger(self.startup)
            return lambda: setattr(self, "bare_ran", True)

        @xronos.reaction_with_deadline(deadline=datetime.timedelta(milliseconds=10))
        def with_deadline(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
            ctx.add_trigger(self.startup)
            return lambda: setattr(self, "with_deadline_ran", True)

    env = xronos.Environment(fast=True, timeout=datetime.timedelta(milliseconds=1))
    reactor = env.create_reactor("forms", Reactor)
    env.execute()

    assert reactor.bare_ran
    assert reactor.with_deadline_ran

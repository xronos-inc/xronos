# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Tests for the experimental dynamic element creation API."""

import datetime
import gc
import weakref
from typing import Callable

import pytest
from typing_extensions import assert_type

import xronos
import xronos.experimental


def ms(milliseconds: int) -> datetime.timedelta:
    return datetime.timedelta(milliseconds=milliseconds)


class AllElements(xronos.Reactor):
    """Creates one of each dynamic element type in its constructor."""

    def __init__(self) -> None:
        super().__init__()
        self.in_port = xronos.experimental.InputPort[int]("in_port", self)
        self.out_port = xronos.experimental.OutputPort[int]("out_port", self)
        self.timer = xronos.experimental.PeriodicTimer("timer", self, period=ms(100))
        self.prog = xronos.experimental.ProgrammableTimer[int]("prog", self)
        self.phys = xronos.experimental.PhysicalEvent[int]("phys", self)
        self.metric = xronos.experimental.Metric(
            "metric", self, description="a metric", unit="widgets"
        )


def test_create_all_element_types() -> None:
    env = xronos.Environment()
    reactor = env.create_reactor("all", AllElements)

    assert reactor.in_port.name == "in_port"
    assert reactor.out_port.name == "out_port"
    assert reactor.timer.name == "timer"
    assert reactor.prog.name == "prog"
    assert reactor.phys.name == "phys"
    assert reactor.metric.name == "metric"

    # Names are qualified by the containing reactor.
    assert reactor.in_port.fqn == "all.in_port"
    assert reactor.timer.fqn == "all.timer"

    # Type-specific accessors keep working on the dynamically created elements.
    assert reactor.timer.period == ms(100)
    assert reactor.metric.description == "a metric"
    assert reactor.metric.unit == "widgets"


class Sender(xronos.Reactor):
    """Sends an increasing counter through a dynamically created output port.

    The output port and the periodic timer are created dynamically, while the
    reaction that drives them is declared statically -- exercising that the two
    APIs compose.
    """

    def __init__(self, period: datetime.timedelta) -> None:
        super().__init__()
        self.output = xronos.experimental.OutputPort[int]("output", self)
        self._timer = xronos.experimental.PeriodicTimer("timer", self, period=period)
        self._count = 0

    @xronos.reaction
    def on_timer(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self._timer)
        output = ctx.add_effect(self.output)

        def handler() -> None:
            self._count += 1
            output.set(self._count)

        return handler


class Receiver(xronos.Reactor):
    """Records values received on a dynamically created input port."""

    def __init__(self) -> None:
        super().__init__()
        self.input_ = xronos.experimental.InputPort[int]("input_", self)
        self.received: list[int] = []

    @xronos.reaction
    def on_input(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        input_ = ctx.add_trigger(self.input_)

        def handler() -> None:
            self.received.append(input_.get())

        return handler


def test_dynamic_ports_end_to_end() -> None:
    env = xronos.Environment(fast=True, timeout=ms(550))
    sender = env.create_reactor("sender", Sender, ms(100))
    receiver = env.create_reactor("receiver", Receiver)
    env.connect(sender.output, receiver.input_)
    env.execute()

    # The dynamically created/connected ports actually carried messages.
    assert receiver.received
    assert receiver.received == sorted(receiver.received)
    assert receiver.received[0] == 1


class SelfLoopingClock(xronos.Reactor):
    """Self-schedules a dynamically created programmable timer."""

    def __init__(self, period: datetime.timedelta) -> None:
        super().__init__()
        self._period = period
        self._timer = xronos.experimental.ProgrammableTimer[None]("timer", self)
        self.ticks = 0

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self.startup)
        effect = ctx.add_effect(self._timer)
        return lambda: effect.schedule(value=None, delay=self._period)

    @xronos.reaction
    def on_timer(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self._timer)
        effect = ctx.add_effect(self._timer)

        def handler() -> None:
            self.ticks += 1
            effect.schedule(value=None, delay=self._period)

        return handler


def test_dynamic_programmable_timer() -> None:
    env = xronos.Environment(fast=True, timeout=ms(350))
    clock = env.create_reactor("clock", SelfLoopingClock, ms(100))
    env.execute()
    assert clock.ticks >= 1


class TypedReactor(xronos.Reactor):
    """Validates that ``[T]`` subscription yields a precisely typed element."""

    def __init__(self) -> None:
        super().__init__()
        in_port = xronos.experimental.InputPort[int]("in_port", self)
        assert_type(in_port, xronos.experimental.InputPort[int])
        # A dynamically created element is a subtype of its core counterpart,
        # so it satisfies the existing public annotations.
        widened: xronos.InputPort[int] = in_port
        del widened


def test_generic_typing() -> None:
    # Constructing the reactor exercises the assert_type / assignment above at
    # runtime; the actual type checking is performed by pyright and mypy.
    xronos.Environment().create_reactor("typed", TypedReactor)


def test_duplicate_dynamic_name() -> None:
    class TwoSamePorts(xronos.Reactor):
        def __init__(self) -> None:
            super().__init__()
            _ = xronos.experimental.InputPort[int]("dup", self)
            _ = xronos.experimental.InputPort[int]("dup", self)

    env = xronos.Environment()
    with pytest.raises(xronos.DuplicateNameError):
        env.create_reactor("two", TwoSamePorts)


def test_duplicate_with_declared_element() -> None:
    class Clashing(xronos.Reactor):
        existing = xronos.InputPortDeclaration[int]()

        def __init__(self) -> None:
            super().__init__()
            # Collides with the statically declared ``existing`` port.
            _ = xronos.experimental.OutputPort[int]("existing", self)

    env = xronos.Environment()
    with pytest.raises(xronos.DuplicateNameError):
        env.create_reactor("clash", Clashing)


def test_duplicate_with_builtin_name() -> None:
    class ClashStartup(xronos.Reactor):
        def __init__(self) -> None:
            super().__init__()
            # Collides with the built-in ``startup`` element.
            _ = xronos.experimental.PeriodicTimer("startup", self)

    env = xronos.Environment()
    with pytest.raises(xronos.DuplicateNameError):
        env.create_reactor("clash_startup", ClashStartup)


def test_parent_keeps_dynamic_element_alive() -> None:
    refs: list[weakref.ref[xronos.experimental.PeriodicTimer]] = []

    class DropsReference(xronos.Reactor):
        def __init__(self) -> None:
            super().__init__()
            timer = xronos.experimental.PeriodicTimer("timer", self, period=ms(100))
            refs.append(weakref.ref(timer))
            # Intentionally keep no strong reference to ``timer``.

    env = xronos.Environment()
    reactor = env.create_reactor("drops", DropsReference)
    gc.collect()

    # The parent reactor registered the element, so the Python wrapper survives
    # even though the constructor kept no reference to it.
    assert refs[0]() is not None
    del reactor

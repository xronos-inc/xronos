# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from collections.abc import Callable

import pytest  # pyright: ignore

import xronos


class Inner(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()
    output = xronos.OutputPortDeclaration[int]()


class Source(xronos.Reactor):
    output = xronos.OutputPortDeclaration[int]()


class Nested(xronos.Reactor):
    def __init__(self) -> None:
        super().__init__()
        self.inner = self.create_reactor("inner", Inner)


class BadTrigger(xronos.Reactor):
    """A reactor that triggers on a child's input instead of its output."""

    def __init__(self) -> None:
        super().__init__()
        self.inner = self.create_reactor("inner", Inner)

    @xronos.reaction
    def bad(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.inner.input_)

        def handler() -> None:
            pass

        return handler


class BadEffect(xronos.Reactor):
    """A reactor that writes a child's output instead of its input."""

    def __init__(self) -> None:
        super().__init__()
        self.inner = self.create_reactor("inner", Inner)

    @xronos.reaction
    def bad(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)
        ctx.add_effect(self.inner.output)

        def handler() -> None:
            pass

        return handler


def test_connection_beyond_direct_child_is_rejected() -> None:
    env = xronos.Environment(fast=True)
    nested = env.create_reactor("nested", Nested)
    source = env.create_reactor("source", Source)
    with pytest.raises(
        xronos.ValidationError, match="does not respect the reactor hierarchy"
    ):
        env.connect(source.output, nested.inner.input_)


def test_trigger_on_child_input_is_rejected() -> None:
    env = xronos.Environment(fast=True)
    env.create_reactor("bad", BadTrigger)
    with pytest.raises(xronos.ValidationError, match="as a trigger"):
        env.execute()


def test_effect_on_child_output_is_rejected() -> None:
    env = xronos.Environment(fast=True)
    env.create_reactor("bad", BadEffect)
    with pytest.raises(xronos.ValidationError, match="as an effect"):
        env.execute()

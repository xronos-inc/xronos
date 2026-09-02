# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from collections.abc import Callable

import pytest  # pyright: ignore

import xronos


class PortReadback(xronos.Reactor):
    """A reactor that writes its own input port and triggers on it."""

    input_ = xronos.InputPortDeclaration[int]()

    @xronos.reaction
    def write(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)
        input_ = ctx.add_effect(self.input_)

        def handler() -> None:
            input_.set(11)

        return handler

    @xronos.reaction
    def read(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        input_ = ctx.add_trigger(self.input_)

        def handler() -> None:
            input_.get()

        return handler


def test_port_readback_is_rejected() -> None:
    env = xronos.Environment(fast=True)
    env.create_reactor("readback", PortReadback)
    with pytest.raises(xronos.ValidationError):
        env.execute()

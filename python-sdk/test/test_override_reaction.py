# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import xronos


class Base(xronos.Reactor):
    """A base reactor with a startup reaction that records 'base' in a log."""

    log: list[str]

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        def handler() -> None:
            self.log.append("base")

        return handler


class Override(Base):
    """A child reactor that overrides the parent's on_startup reaction."""

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        def handler() -> None:
            self.log.append("override")

        return handler


def test_override_reaction() -> None:
    """Overriding a parent reaction replaces it; only the child's handler runs."""
    log: list[str] = []
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("override", Override)
    reactor.log = log
    env.execute()
    assert log == ["override"], (
        f"Expected only the overriding reaction to run, but got: {log}"
    )


if __name__ == "__main__":
    test_override_reaction()

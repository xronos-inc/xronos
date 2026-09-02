# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from collections.abc import Callable

import pytest  # pyright: ignore

import xronos


class EarlyWriter(xronos.Reactor):
    """A reactor that illegally writes through an effect during assembly.

    A reaction declaration body runs during assembly, before execution
    starts. Writing through an effect there must raise a ``ValidationError``.
    """

    output = xronos.OutputPortDeclaration[int]()
    timer = xronos.ProgrammableTimerDeclaration[int]()
    metric = xronos.MetricDeclaration(description="test metric")

    def __init__(self, action: str) -> None:
        super().__init__()
        self._action = action

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        # Execution has not started while the declaration body runs, so each
        # of these writes is illegal.
        if self._action == "set":
            ctx.add_effect(self.output).set(42)
        elif self._action == "schedule":
            ctx.add_effect(self.timer).schedule(42)
        elif self._action == "record":
            ctx.add_effect(self.metric).record(1)
        elif self._action == "trigger_shutdown":
            ctx.add_effect(self.shutdown).trigger_shutdown()

        def handler() -> None:
            pass

        return handler


@pytest.mark.parametrize(
    ("action", "message"),
    [
        ("set", "Port early.output may not be set before execution has started"),
        (
            "schedule",
            "Programmable timer early.timer may not be scheduled"
            " before execution has started",
        ),
        (
            "record",
            "Metric early.metric may not be recorded before execution has started",
        ),
        (
            "trigger_shutdown",
            "Shutdown early.shutdown may not be triggered before execution has started",
        ),
    ],
)
def test_write_before_start(action: str, message: str) -> None:
    env = xronos.Environment(fast=True)
    env.create_reactor("early", EarlyWriter, action)
    with pytest.raises(xronos.ValidationError, match=message):
        env.execute()

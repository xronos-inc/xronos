# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from collections.abc import Callable

import pytest  # pyright: ignore

import xronos


class LateDeclarer(xronos.Reactor):
    """A reactor that illegally declares a trigger/effect from its handler.

    Triggers and effects must be declared during assembly, before execution
    starts. Declaring one from a running handler must raise a
    ``ValidationError``.
    """

    input_ = xronos.InputPortDeclaration[int]()
    output = xronos.OutputPortDeclaration[int]()
    timer = xronos.ProgrammableTimerDeclaration[int]()
    metric = xronos.MetricDeclaration(description="test metric")

    def __init__(self, action: str) -> None:
        super().__init__()
        self._action = action

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)

        def handler() -> None:
            # Execution has started by the time the handler runs, so each of
            # these declarations is illegal.
            if self._action == "trigger":
                ctx.add_trigger(self.input_)
            elif self._action == "port_effect":
                ctx.add_effect(self.output)
            elif self._action == "timer_effect":
                ctx.add_effect(self.timer)
            elif self._action == "metric_effect":
                ctx.add_effect(self.metric)
            elif self._action == "shutdown_effect":
                ctx.add_effect(self.shutdown)

        return handler


@pytest.mark.parametrize(
    "action",
    ["trigger", "port_effect", "timer_effect", "metric_effect", "shutdown_effect"],
)
def test_declare_after_start(action: str) -> None:
    env = xronos.Environment(fast=True)
    env.create_reactor("late", LateDeclarer, action)
    with pytest.raises(
        xronos.ValidationError, match="may not be declared once execution has started"
    ):
        env.execute()

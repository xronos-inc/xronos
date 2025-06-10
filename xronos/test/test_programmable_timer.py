# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos


class Clock(xronos.Reactor):
    _programmable_timer = xronos.ProgrammableTimerDeclaration[None]()

    def __init__(self, period: datetime.timedelta) -> None:
        super().__init__()
        self._period = period

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.startup)
        action = interface.add_effect(self._programmable_timer)

        return lambda: action.schedule(value=None, delay=self._period)

    @xronos.reaction
    def on_action(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self._programmable_timer)
        internal_event_effect = interface.add_effect(self._programmable_timer)

        def handler() -> None:
            print(f"{self._programmable_timer.fqn} triggered")
            internal_event_effect.schedule(value=None, delay=self._period)

        return handler


def run(env: xronos.Environment) -> None:
    env.create_reactor("clock", Clock, datetime.timedelta(seconds=1))
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    run(env)


def test_action() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    run(env)


if __name__ == "__main__":
    main()

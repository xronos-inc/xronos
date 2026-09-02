# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from collections.abc import Callable

import xronos


class MainReactor(xronos.Reactor):
    _colliding_event = xronos.ProgrammableTimerDeclaration[bool]()
    _request_shutdown = xronos.ProgrammableTimerDeclaration[None]()
    _after_shutdown = xronos.ProgrammableTimerDeclaration[None]()

    def __init__(self) -> None:
        super().__init__()
        self.shutdown_fired = False
        self.colliding_event_fired = False

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self.startup)
        request_shutdown_effect = ctx.add_effect(self._request_shutdown)
        after_shutdown_effect = ctx.add_effect(self._after_shutdown)

        def handler() -> None:
            request_shutdown_effect.schedule(None, datetime.timedelta(seconds=1))
            after_shutdown_effect.schedule(None, datetime.timedelta(seconds=2))

        return handler

    @xronos.reaction
    def on_request_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        _ = ctx.add_trigger(self._request_shutdown)
        colliding_event = ctx.add_effect(self._colliding_event)
        shutdown_effect = ctx.add_effect(self.shutdown)

        def handler() -> None:
            # The zero-delay event lands on the requested shutdown tag. Only
            # shutdown events run there, so it must be dropped.
            shutdown_effect.trigger_shutdown()
            colliding_event.schedule(True)

        return handler

    @xronos.reaction
    def on_shutdown_or_colliding_event(
        self, ctx: xronos.ReactionContext
    ) -> Callable[[], None]:
        shutdown = ctx.add_trigger(self.shutdown)
        colliding_event = ctx.add_trigger(self._colliding_event)

        def handler() -> None:
            if shutdown.is_present():
                self.shutdown_fired = True
            if colliding_event.is_present():
                self.colliding_event_fired = True
            print(f"Stopping at {ctx.current_time}")

        return handler


def run(env: xronos.Environment) -> None:
    main_reactor = env.create_reactor("main", MainReactor)
    env.execute()
    assert main_reactor.shutdown_fired
    assert not main_reactor.colliding_event_fired


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    run(env)


def test_stop() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    run(env)


if __name__ == "__main__":
    main()

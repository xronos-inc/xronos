# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

import xronos


class MainReactor(xronos.Reactor):
    check_shutdown = xronos.ProgrammableTimerDeclaration[bool]()
    _request_shutdown = xronos.ProgrammableTimerDeclaration[None]()
    _after_shutdown = xronos.ProgrammableTimerDeclaration[None]()

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        _ = interface.add_trigger(self.startup)
        request_shutdown_effect = interface.add_effect(self._request_shutdown)
        after_shutdown_effect = interface.add_effect(self._after_shutdown)

        def handler() -> None:
            request_shutdown_effect.schedule(None, datetime.timedelta(seconds=1))
            after_shutdown_effect.schedule(None, datetime.timedelta(seconds=2))

        return handler

    @xronos.reaction
    def on_request_shutdown(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        _ = interface.add_trigger(self._request_shutdown)
        check_shutdown = interface.add_effect(self.check_shutdown)
        shutdown_effect = interface.add_effect(self.shutdown)

        def handler() -> None:
            shutdown_effect.trigger_shutdown()
            check_shutdown.schedule(True)

        return handler

    @xronos.reaction
    def on_shutdown_or_check_shutdown(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        shutdown = interface.add_trigger(self.shutdown)
        check_shutdown = interface.add_trigger(self.check_shutdown)

        def handler() -> None:
            if (not shutdown.is_present()) and check_shutdown.is_present():
                raise Exception("Shutdown was not triggered at the expected tag")
            elif shutdown.is_present() and not check_shutdown.is_present():
                raise Exception(
                    "Shutdown occurred but check_shutdown was not triggered"
                )
            else:
                print(f"Success: stopping at {self.get_time()}")

        return handler


def run(env: xronos.Environment) -> None:
    env.create_reactor("main", MainReactor)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    run(env)


def test_stop() -> None:
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=5))
    run(env)


if __name__ == "__main__":
    main()

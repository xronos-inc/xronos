# SPDX-FileCopyrightText: (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Reactor to manage the lifecycle of the Keyboard Synth application."""

# pyright: standard

import datetime
from typing import Callable

import xronos

from reactor_log import log


class LifecycleManager(xronos.Reactor):
    # when present, `stop` tells the lifecycle manager to ask all services to stop
    stop = xronos.InputPortDeclaration[None]()
    # signal services to stop
    stop_requested = xronos.OutputPortDeclaration[None]()

    # user sends a SIGINT (requires the signal be captured and attached to this event)
    user_interrupt = xronos.PhysicalEventDeclaration[None]()

    # shutdown synchronization -- feedback from services that they have stopped.
    # service reacotrs will always send a stop signal after a gracefull shutdown,
    # and may send a stop signal upon abnormal termination (if caught).
    audio_stopped = xronos.InputPortDeclaration[None]()
    websocket_server_stopped = xronos.InputPortDeclaration[None]()
    webserver_stopped = xronos.InputPortDeclaration[None]()

    # have all services stopped gracefully?
    stopped_gracefully = False
    stop_timeout = datetime.timedelta(seconds=5)
    __stop_timer = xronos.ProgrammableTimerDeclaration[None]()

    def __init__(self) -> None:
        super().__init__()
        self.__audio_stopped = False
        self.__websocket_server_stopped = False
        self.__webserver_stopped = False

    @xronos.reaction
    def __on_stop_requested(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Signal all services to stop. Start timeout for service shutdown."""
        interface.add_trigger(self.user_interrupt)
        interface.add_trigger(self.stop)
        stop_requested = interface.add_effect(self.stop_requested)
        stop_timer_effect = interface.add_effect(self.__stop_timer)

        def handler() -> None:
            log(self, "stop requested")
            stop_requested.set(None)
            stop_timer_effect.schedule(None, self.stop_timeout)

        return handler

    @xronos.reaction
    def __on_stopped(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Handle inputs from upstream services that they have stopped.

        Upon receipt of shutdown signals from all services, initiate shutdown.
        """
        audio_stopped_trigger = interface.add_trigger(self.audio_stopped)
        websocket_server_stopped_trigger = interface.add_trigger(
            self.websocket_server_stopped
        )
        webserver_stopped_trigger = interface.add_trigger(self.webserver_stopped)

        def handler() -> None:
            if audio_stopped_trigger.is_present():
                self.__audio_stopped = True
            if websocket_server_stopped_trigger.is_present():
                self.__websocket_server_stopped = True
            if webserver_stopped_trigger.is_present():
                self.__webserver_stopped = True
            if (
                self.__audio_stopped
                and self.__websocket_server_stopped
                and self.__webserver_stopped
            ):
                self.stopped_gracefully = True
                log(self, "requesting shutdown")
                self.request_shutdown()

        return handler

    @xronos.reaction
    def __on_stop_timeout(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.__stop_timer)

        def handler() -> None:
            if not self.stopped_gracefully:
                timeout_s = self.stop_timeout.seconds
                log(self, f"warning: services timed out after {timeout_s} seconds")
                log(self, "requesting shutdown")
                self.request_shutdown()

        return handler

    @xronos.reaction
    def __on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)
        return lambda: log(self, "stopped")

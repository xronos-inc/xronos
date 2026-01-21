# SPDX-FileCopyrightText: (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

import threading
from concurrent.futures import Future
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler
from typing import Callable

import xronos

from reactor_log import log


class Webserver(xronos.Reactor):
    """Webserver reactor for the Keyboard Synth example.

    This reactor hosts a simple HTTP server with static content.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8000) -> None:
        super().__init__()

        request_handler = partial(SimpleHTTPRequestHandler, directory="static")
        self.__server = HTTPServer((host, port), request_handler, False)
        self.__server_thread = threading.Thread(target=self.__run_server)
        self.__server_stop_thread = threading.Thread(target=self.__stop_server)
        self.__server_stop_thread_started: Future[bool] = Future()

    def __run_server(self) -> None:
        """Thread that runs the HTTP server."""
        host = self.__server.server_address[0]
        if isinstance(host, bytes):
            host = host.decode()
        self.__async_log.trigger(
            f"started on http://{host}:{self.__server.server_port}"
        )
        self.__server.serve_forever()
        self.__async_log.trigger("stopping")
        self.__stopped_callback.trigger(None)

    def __stop_server(self) -> None:
        """Thread that initiates server shutdown and blocks until complete."""
        self.__server_stop_thread_started.set_result(True)
        self.__server.shutdown()  # blocks until shutdown

    ####################
    # service lifecycle
    ####################

    # program lifecycle management
    request_service_stop = xronos.InputPortDeclaration[None]()
    service_stopped = xronos.OutputPortDeclaration[None]()
    __stopped_callback = xronos.PhysicalEventDeclaration[None]()

    @xronos.reaction
    def __startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        stopped_effect = interface.add_effect(self.service_stopped)

        def handler() -> None:
            try:
                self.__server.server_bind()
                self.__server.server_activate()
                self.__server_thread.start()
            except Exception as e:
                log(self, f"failed to start: {e}")
                stopped_effect.set(None)

        return handler

    @xronos.reaction
    def __on_request_service_stop(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Signal shutdown of the websocket server."""
        interface.add_trigger(self.request_service_stop)
        return lambda: self.__server_stop_thread.start()

    @xronos.reaction
    def __on_stopped_callback(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Callback issued when websocket server has stopped."""
        interface.add_trigger(self.__stopped_callback)
        stopped_effect = interface.add_effect(self.service_stopped)

        def handler() -> None:
            log(self, "stopped")
            stopped_effect.set(None)

        return handler

    @xronos.reaction
    def __on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Stop server and join all threads on shutdown."""
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            if self.__server_thread.is_alive():
                log(self, "warning: service was not stopped before shutdown")
                if (
                    not self.__server_stop_thread.is_alive()
                    and not self.__server_stop_thread_started.done()
                ):
                    self.__server_stop_thread.start()

            if (
                self.__server_stop_thread.is_alive()
                or self.__server_stop_thread_started.done()
            ):
                self.__server_stop_thread.join()
            self.__server_thread.join()

        return handler

    ##################
    # logging
    ##################

    # log from threads not managed by the xronos runtime
    __async_log = xronos.PhysicalEventDeclaration[str]()

    @xronos.reaction
    def __on_async_log(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        async_log = interface.add_trigger(self.__async_log)
        return lambda: log(self, async_log.get())

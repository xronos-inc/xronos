# SPDX-FileCopyrightText: (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Websocket server reactor for the Keyboard Synth example."""

import copy
import json
import threading
from typing import Any, Callable, Set

import websockets.exceptions as ws_exceptions
import websockets.sync.server as ws_server
import xronos
from websockets.frames import CLOSE_CODE_EXPLANATIONS, CloseCode

from keypress import Keypress
from reactor_log import log


class WebsocketServer(xronos.Reactor):
    """Websocket server to receive commands from a web-hosted user interface.

    This class hosts a websocket server that receives keypress events from a
    web UI and publishes them to its output port.

    === Keyboard Protocol ===

    Clients must send JSON-formatted messages. The expected message schema is:

    {
        "type": "down" | "up" | "exit",  # Required
        "note": "<string>"               # Required for "down"/"up"
    }

    - type == "down": Indicates a key has been pressed.
        -> Triggers the `keypress` output with enabled=True.

    - type == "up": Indicates a key has been released.
        -> Triggers the `keypress` output with enabled=False.

    - type == "exit": Requests termination of the session.
        -> Triggers the `command_exit` output with value None.

    Invalid or malformed messages are silently dropped and logged.

    === Output Ports ===

    - keypress: OutputPort[Keypress]
        Emitted when a valid keypress event is received.
        Carries the following Keypress structure:
            - client_id: int (unique per websocket client)
            - note: str (note identifier from UI)
            - enabled: bool (True if "down", False if "up")

    - command_exit: OutputPort[None]
        Emitted when a message of type "exit" is received.

    - service_stopped: OutputPort[None]
        Emitted when the server stops, either from shutdown or service failure.

    === Input Ports ===

    - request_service_stop: InputPort[None]
        Request to stop the websocket server (graceful shutdown).
        Will invoke shutdown of the server thread and notify clients.

    === Service Lifecycle ===

    - Startup:
        Triggered by Xronos startup.
        Launches a websocket server on a background thread, listening on `host:port`.

    - Shutdown:
        Triggered by Xronos shutdown or request_service_stop.
        Closes all active connections and joins the server thread.

    === Threading and Safety ===

    - Each WebSocket client is handled in its own thread.
    - Logging from non-reactor threads is routed via `__async_log`.

    === Exceptions ===

    - Invalid messages or connection errors do not crash the server.
      Errors are caught, logged, and connections are cleaned up gracefully.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8001) -> None:
        super().__init__()

        self.__host = host
        self.__port = port
        self.__server: ws_server.Server  # initiliazed at startup
        self.__server_thread = threading.Thread(target=self.__run_server)
        self._clients: Set[ws_server.ServerConnection] = set()

    ###################
    # websocket server
    ###################

    def __run_server(self) -> None:
        """Thread that runs the websocket server instance."""
        self.__async_log.trigger(f"started on ws://{self.__host}:{self.__port}")
        self.__server.serve_forever()
        self.__async_log.trigger("stopping")
        self.__stopped_callback.trigger(None)

    def __client_handler(self, conn: ws_server.ServerConnection) -> None:
        """One client handler thread per client."""

        class ConnectionScope:
            """Websocket connection scope manages graceful startup and shutdown."""

            def __init__(
                self, outer: WebsocketServer, conn: ws_server.ServerConnection
            ) -> None:
                self.conn = conn
                self.client_id = hash(conn)
                self.outer = outer
                self.remote = ("unknown", 0)
                self.close_code = CloseCode.INTERNAL_ERROR

            def __enter__(self) -> "ConnectionScope":
                self.outer._clients.add(self.conn)
                self.remote = copy.deepcopy(self.conn.remote_address)
                return self

            def __exit__(
                self,
                exc_type: type[BaseException] | None,
                exc_val: BaseException | None,
                exc_tb: object | None,
            ) -> None:
                try:
                    self.conn.close(
                        self.close_code,
                        CLOSE_CODE_EXPLANATIONS.get(self.close_code, "closing"),
                    )
                except ws_exceptions.ConnectionClosed:
                    pass
                except Exception as e:
                    self.outer.__async_log.trigger(
                        f"failed to close client connection: {e}"
                    )
                finally:
                    self.outer._clients.discard(self.conn)

        # process messages until connection closes
        with ConnectionScope(self, conn) as scope:
            try:
                self.__async_log.trigger(
                    f"client connected: {scope.remote[0]}:{scope.remote[1]}"
                )
                for message in scope.conn:
                    msg = message
                    if isinstance(msg, bytes):
                        msg = msg.decode("utf-8")
                    self.__process_message(scope.client_id, msg)
                scope.close_code = CloseCode.NORMAL_CLOSURE
                self.__async_log.trigger(
                    f"client disconnected: {scope.remote[0]}:{scope.remote[1]}"
                )
            except ws_exceptions.WebSocketException as e:
                self.__async_log.trigger(f"client terminated: {e}")
                scope.close_code = CloseCode.ABNORMAL_CLOSURE
            except Exception as e:
                self.__async_log.trigger(f"unhandled client exception: {e}")
                scope.close_code = CloseCode.INTERNAL_ERROR

    #################
    # messages
    #################

    keypress = xronos.OutputPortDeclaration[Keypress]()
    __keypress_trigger = xronos.PhysicalEventDeclaration[Keypress]()

    def __process_message(self, client_id: int, message: str) -> None:
        """Asynchronous helper to process incoming messages.

        This method is called in the context of a websocket client thread
        and raises physical events in response to user commands.
        """
        try:
            data: dict[str, Any] = json.loads(message)

            msg_type = data.get("type")
            if not isinstance(msg_type, str):
                return
            elif msg_type == "exit":
                self.__command_exit_trigger.trigger(None)
                return

            note = data.get("note")
            if isinstance(note, str):
                self.__keypress_trigger.trigger(
                    Keypress(client_id=client_id, note=note, enabled=msg_type == "down")
                )
                return
        except Exception as e:
            self.__async_log.trigger(f"Failed to parse message: {e}")

    @xronos.reaction
    def __on_keypress_event(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        trigger = interface.add_trigger(self.__keypress_trigger)
        effect = interface.add_effect(self.keypress)
        return lambda: effect.set(trigger.get())

    command_exit = xronos.OutputPortDeclaration[None]()
    __command_exit_trigger = xronos.PhysicalEventDeclaration[None]()

    @xronos.reaction
    def __on_command_exit(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.__command_exit_trigger)
        effect = interface.add_effect(self.command_exit)

        def handler() -> None:
            log(self, "command: exit")
            effect.set(None)

        return handler

    ####################
    # service lifecycle
    ####################

    # program lifecycle management
    request_service_stop = xronos.InputPortDeclaration[None]()
    service_stopped = xronos.OutputPortDeclaration[None]()
    __stopped_callback = xronos.PhysicalEventDeclaration[None]()

    @xronos.reaction
    def __startup(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        stopped_effect = interface.add_effect(self.service_stopped)

        def handler() -> None:
            try:
                self.__server = ws_server.serve(
                    self.__client_handler, self.__host, self.__port
                )
                self.__server_thread.start()
            except Exception as e:
                log(self, f"failed to start: {e}")
                stopped_effect.set(None)

        return handler

    @xronos.reaction
    def __on_request_service_stop(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.request_service_stop)

        def handler() -> None:
            self.__server.shutdown()

        return handler

    @xronos.reaction
    def __on_stopped_callback(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.__stopped_callback)
        stopped_effect = interface.add_effect(self.service_stopped)

        def handler() -> None:
            log(self, "stopped")
            stopped_effect.set(None)

        return handler

    @xronos.reaction
    def __on_shutdown(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            # stop server
            if self.__server_thread.is_alive():
                log(self, "warning: service was not stopped before shutdown")
                self.__server.shutdown()

            # drop clients
            # thread safety: list(self._clients) snapshots and `close()` is idempotent
            for conn in list(self._clients):
                try:
                    conn.close(
                        CloseCode.GOING_AWAY,
                        CLOSE_CODE_EXPLANATIONS[CloseCode.GOING_AWAY],
                    )
                except ws_exceptions.ConnectionClosed:
                    pass
                except Exception as e:
                    log(self, f"client dropped: {e}")

            self.__server_thread.join()

        return handler

    ##################
    # logging
    ##################

    # log from threads not managed by the xronos runtime
    __async_log = xronos.PhysicalEventDeclaration[str]()

    @xronos.reaction
    def __on_async_log(  # pyright: ignore[reportUnusedFunction]
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        async_log = interface.add_trigger(self.__async_log)
        return lambda: log(self, async_log.get())

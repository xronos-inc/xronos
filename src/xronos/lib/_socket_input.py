# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import queue
import socket
import socketserver
import threading
import time
from collections.abc import Generator
from enum import Enum

from xronos.lib import ExternalInput


class SocketInput(ExternalInput[bytes]):
    """A reactor that reads inputs from a network socket.

    This reactor opens a network socket and reads bytes from it. The bytes are forwarded
    to the output port. The reactor can be configured to open either a TCP
    server, a TCP client or a UDP listener. Deserialization of the data must be handled
    by the reactor connected to the output port.

    In TCP client mode, if ``reconnect`` is set to True, the reactor will keep trying to
    connect to the server if it gets a :exc:`ConnectionRefusedError`. Once connected, it
    will read data until the server closes the connection, at which point it will try to
    reconnect, if ``reconnect`` is set to True. ``reconnect_interval`` is the interval
    between reconnection attempts. If ``reconnect_only_first_time`` is set to True, then
    reconnects are only performed the first time the connection is opened.

    In TCP server mode, the reactor will open a server socket and accept incoming
    connections. It will read data from each client until the connection is closed. If
    multiple clients are connected, then the data from the different clients will appear
    interleaved on the single output port ``output``.

    In UDP listener mode, the reactor will open a socket and listen for incoming data
    coming from any client.

    Args:
        host: The host to connect to or bind to.
        port: The port to connect to or bind to.
        mode: The mode of operation. It can be one of the following: "tcp_server",
            "tcp_client" or "udp_listener".
        buffer_size: The size of the buffer used to read data from the socket.
        reconnect: Whether to reconnect to the server if the connection is closed. This
            is only applicable in TCP client mode.
        reconnect_interval: The interval in seconds between reconnection attempts. This
            is only applicable in TCP client mode.
        reconnect_only_first_time: Whether to reconnect only the first time the
            connection is opened. This is only applicable in TCP client mode.

    Attributes:
        output(xronos.OutputPort[T]): A port that forwards any bytes received
            on the socket.
    """

    class Mode(Enum):
        TCP_SERVER = "tcp_server"
        TCP_CLIENT = "tcp_client"
        UDP_LISTENER = "udp_listener"

    def __init__(  # noqa: PLR0913
        self,
        host: str,
        port: int,
        mode: "SocketInput.Mode",
        buffer_size: int = 1024,
        reconnect: bool = True,
        reconnect_interval: float = 0.5,
        reconnect_only_first_time: bool = False,
    ):
        super().__init__(read_input=self.__read_socket_input())

        if mode not in SocketInput.Mode.__members__.values():
            raise ValueError(
                f"Invalid mode: {mode}. Supported modes are {list(SocketInput.Mode)}."
            )

        if reconnect_only_first_time and reconnect:
            raise ValueError(
                "`reconnect_only_first_time` and `reconnect` cannot both be true."
            )

        self.__host = host
        self.__port = port
        self.__buffer_size = buffer_size
        self.__reconnect = reconnect
        self.__reconnect_interval = reconnect_interval
        self.__reconnect_only_first_time = reconnect_only_first_time
        self.__mode = mode

    # The socketserver library is used to create a TCP server that can handle multiple
    # clients concurrently using threads. It requires defining some simple classes.
    # See the following example:
    # https://docs.python.org/3/library/socketserver.html#asynchronous-mixins
    class __ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
        # To avoid the server from hanging on Ctrl+C or other exceptions in the main
        # thread, we use daemon threads which are killed when the main thread exits.
        # See: https://stackoverflow.com/a/48283153
        daemon_threads = True

        # Since we use daemon threads, there is a risk that the server socket is not
        # closed properly. To avoid getting "Address already in use" errors we
        # explicitly allow the reuse of the address.
        allow_reuse_address = True
        pass

    class __RequestHandler(socketserver.BaseRequestHandler):
        def handle(self) -> None:
            """Handle a client connection by reading out data until it is closed.

            This method is called when a client connects to the TCP server. It is run in
            a separate daemonic thread. The thread only stops when the client closes the
            connection, or when the main thread runtime thread stops.
            """
            while True:
                # The buffer size is set as an attribute of the server object.
                data = self.request.recv(self.server.buffer_size)  # type: ignore
                if data:
                    # The data is put into the receive_queue which is an attribute of
                    # the server object.
                    self.server.receive_queue.put(data)  # type: ignore
                else:
                    break

    def __read_tcp_server(self) -> Generator[bytes]:
        """Open a TCP server and yield the received data.

        The socketserver library is used and a TCP server is started in a separate
        thread. A Queue is used to communicate between the TCP server threads and the
        generator.

        Yields:
            bytes: Data received from the client.
        """
        receive_queue: queue.Queue[bytes] = queue.Queue()
        with self.__ThreadedTCPServer(
            (self.__host, self.__port), self.__RequestHandler
        ) as server:
            server.allow_reuse_port = True
            # TCPServer does not support an easy way of providing arguments to the
            # handler class. A workaround is to set the arguments as attributes of the
            # server object.
            server.receive_queue = receive_queue  # type: ignore
            server.buffer_size = self.__buffer_size  # type: ignore

            server_thread = threading.Thread(target=server.serve_forever, daemon=True)
            try:
                server_thread.start()
                while True:
                    yield receive_queue.get()
            finally:
                # If the generator is stopped, the server must be explicitly shut down.
                server.shutdown()
                server_thread.join()

    def __read_tcp_client(self) -> Generator[bytes]:
        """Connect to a TCP server and yield the received data.

        The low-level socket library is used to connect to a TCP server and read data
        from it. Reconnection is optional, either on every connection close or only the
        first time the connection is opened.

        Yields:
            bytes: Data received from the server.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            outer_loop = True
            while outer_loop:
                try:
                    sock.connect((self.__host, self.__port))
                    while True:
                        data = sock.recv(self.__buffer_size)
                        if data:
                            yield data
                        elif not self.__reconnect:
                            # If connection was closed by server, and reconnect is
                            # disabled, stop the generator.
                            outer_loop = False
                            break
                except ConnectionRefusedError:
                    if self.__reconnect or self.__reconnect_only_first_time:
                        # If reconnect is enabled, wait for a while and try to
                        # reconnect.
                        time.sleep(self.__reconnect_interval)
                        pass
                    else:
                        raise

    def __read_udp_listener(self) -> Generator[bytes]:
        """Open a UDP listener and yield the received data."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.__host, self.__port))
            while True:
                data, _ = sock.recvfrom(self.__buffer_size)
                yield data

    def __read_socket_input(self) -> Generator[bytes]:
        """Opens a socket and yields the received data from it.

        Yields:
            bytes: Data received from the socket(s).

        Returns:
            None
        """
        if self.__mode == SocketInput.Mode.TCP_SERVER:
            yield from self.__read_tcp_server()
        elif self.__mode == SocketInput.Mode.TCP_CLIENT:
            yield from self.__read_tcp_client()
        elif self.__mode == SocketInput.Mode.UDP_LISTENER:
            yield from self.__read_udp_listener()
        else:
            # We should never reach this
            raise RuntimeError("unexpected mode")

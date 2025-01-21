# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import socket
import threading
import time

import xronos
from xronos.lib import SocketInput
from xronos.lib.test.util import AssertList

HOST = "localhost"


def find_free_port() -> int:
    """Find a free port on the local machine."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("localhost", 0))  # Bind to a random free port
        return s.getsockname()[1]


def tcp_client_writer(
    host: str, port: int, data: list[bytes], sleep_between_writes: float = 0.1
) -> None:
    """Connect to a TCP server and write data to it."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        while True:
            try:
                s.connect((host, port))
                for d in data:
                    s.sendall(d)
                    # This sleep is necessary to avoid the server receiving all the data
                    # in a single recv call.
                    time.sleep(sleep_between_writes)
                break
            except ConnectionRefusedError:
                pass


def tcp_server_writer(
    host: str, port: int, data: list[bytes], sleep_between_writes: float = 0.1
) -> None:
    """Open TCP server and write data to a client."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        conn, _ = s.accept()
        with conn:
            for d in data:
                conn.sendall(d)
                # This sleep is necessary to avoid the client receiving all the data
                # in a single recv call.
                time.sleep(sleep_between_writes)


def udp_writer(host: str, port: int, data: list[bytes]) -> None:
    """Open a UDP socket and write data to a listener."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        while True:
            try:
                s.connect((host, port))
                for d in data:
                    s.sendall(d)
                    time.sleep(0.1)
                break
            except ConnectionRefusedError:
                pass


def test_simple_tcp_server() -> None:
    """Test ExternalInput with a generator that produces a list of values."""
    port = find_free_port()
    mode = SocketInput.Mode.TCP_SERVER
    env = xronos.Environment()
    testInput = [b"HelloWorld", b"Testing", b"123"]
    env.connect(
        env.create_reactor("SocketInput", SocketInput, HOST, port, mode).output,
        env.create_reactor(
            "AssertList", AssertList[bytes], testInput, debug=False
        ).input_,
    )

    thread = threading.Thread(
        target=tcp_client_writer, args=(HOST, port, testInput), daemon=True
    )
    thread.start()
    env.execute()
    thread.join()


def test_tcp_server_with_multiple_clients() -> None:
    """Test ExternalInput with a generator that produces a list of values."""
    port = find_free_port()
    mode = SocketInput.Mode.TCP_SERVER
    env = xronos.Environment()
    n_clients = 10
    testInput = [b"HelloWorld" for _ in range(n_clients)]
    env.connect(
        env.create_reactor("SocketInput", SocketInput, HOST, port, mode).output,
        env.create_reactor(
            "AssertList", AssertList[bytes], testInput, debug=False
        ).input_,
    )
    threads = [
        threading.Thread(
            target=tcp_client_writer, args=(HOST, port, [testInput[0]]), daemon=True
        )
        for _ in range(n_clients)
    ]
    for thread in threads:
        thread.start()
    env.execute()
    for thread in threads:
        thread.join()


def test_simple_tcp_client() -> None:
    """Test ExternalInput with a generator that produces a list of values."""
    port = find_free_port()
    mode = SocketInput.Mode.TCP_CLIENT
    env = xronos.Environment()
    testInput = [b"HelloWorld", b"Testing", b"123"]
    env.connect(
        env.create_reactor("SocketInput", SocketInput, HOST, port, mode).output,
        env.create_reactor(
            "AssertList", AssertList[bytes], testInput, debug=False
        ).input_,
    )

    thread = threading.Thread(
        target=tcp_server_writer, args=(HOST, port, testInput), daemon=True
    )
    thread.start()
    env.execute()
    thread.join()


def test_simple_udp_listener() -> None:
    """Test ExternalInput with a generator that produces a list of values."""
    port = find_free_port()
    mode = SocketInput.Mode.UDP_LISTENER
    env = xronos.Environment()
    # Since UDP is connectionless, there is no way to know if the data was received.
    # Therefore we repeat the same data multiple times to increase the chances of
    # the UDP listener starting up and receiving the data. We only require 3 received
    # packets to pass the test.
    testInput = [b"HelloWorld" for _ in range(1000)]
    env.connect(
        env.create_reactor("SocketInput", SocketInput, HOST, port, mode).output,
        env.create_reactor(
            "AssertList", AssertList[bytes], testInput[:3], debug=False
        ).input_,
    )

    thread = threading.Thread(
        target=udp_writer, args=(HOST, port, testInput), daemon=True
    )
    thread.start()
    env.execute()
    # Note that we dont join the thread here so we dont have to wait for the UDP
    # listener to receive all the data. The thread is killed when the test is done.

# SPDX-FileCopyrightText: (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""This application is a standalone version of the Keyboard Synthesizer example."""

# pyright: standard

import argparse

import xronos

from audio_bridge import AudioBridge
from lifecycle_manager import LifecycleManager
from reactor_log import log
from synthesizer import Synthesizer
from webserver import Webserver
from websocket_server import WebsocketServer

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--telemetry",
        dest="telemetry",
        nargs="?",
        const="localhost",
        default=None,
        help="Enable telemetry publishing (optionally specify a host address).",
    )
    parser.add_argument(
        "--host",
        dest="host",
        nargs="?",
        default="0.0.0.0",
        help="Address on which to host the UI webserver.",
    )
    parser.add_argument(
        "--port",
        dest="port",
        nargs="?",
        type=int,
        default=8000,
        help="Starting port for web services.",
    )
    args = parser.parse_args()

    env = xronos.Environment()
    if args.telemetry:
        env.enable_tracing(args.telemetry)

    audio_bridge = env.create_reactor("Audio Bridge", AudioBridge)

    websocket_server = env.create_reactor(
        "Websocket Server",
        WebsocketServer,
        host=args.host,
        port=args.port + 1,
    )

    client_synth = env.create_reactor(
        "Synthesizer", Synthesizer, samplerate=audio_bridge.samplerate, blocksize=1024
    )
    env.connect(client_synth.frames, audio_bridge.input)
    env.connect(websocket_server.keypress, client_synth.keypress)

    webserver = env.create_reactor(
        "Webserver", Webserver, host=args.host, port=args.port
    )

    lifecycle = env.create_reactor(
        "Lifecycle Manager",
        LifecycleManager,
    )

    env.connect(websocket_server.command_exit, lifecycle.stop)
    env.connect(lifecycle.stop_requested, audio_bridge.request_service_stop)
    env.connect(lifecycle.stop_requested, websocket_server.request_service_stop)
    env.connect(lifecycle.stop_requested, webserver.request_service_stop)
    env.connect(audio_bridge.service_stopped, lifecycle.audio_stopped)
    env.connect(websocket_server.service_stopped, lifecycle.websocket_server_stopped)
    env.connect(webserver.service_stopped, lifecycle.webserver_stopped)

    try:
        env.execute()
    except KeyboardInterrupt:
        pass
    finally:
        log(lifecycle, "program complete")

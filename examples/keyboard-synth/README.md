# Keyboard Synthesizer

A reactive, real-time audio synthesis application built using the [Xronos](https://github.com/xronos-inc/xronos)
concurrency framework. This project implements a keyboard synthesizer controlled via a web interface, using Xronos
reactors to achieve deterministic behavior, efficient real-time audio processing, and clean lifecycle management.

## Features

- **Reactive Audio Synthesizer**: Real-time additive synthesis of sine tones per client keypress.
- **WebSocket Control**: Web clients can send key events to trigger audio notes.
- **Audio Streaming**: Frames are buffered and streamed to an output device with underrun monitoring.
- **Web UI**: Static files are served locally to host a user-friendly interface.
- **Graceful Lifecycle**: Full startup/shutdown orchestration with telemetry and metrics support.

## Design Highlights

- **Determinism**: Reactor-based audio generation ensures block-aligned waveform continuity.
- **Metrics**: Includes buffer size and underrun reporting for audio performance introspection.
- **Safety**: Clean thread isolation and graceful teardown via Xronos shutdown hooks.
- **Extensibility**: Modular design for easy substitution or extension of reactors.

## Requirements

### Prerequisites

- xronos python library
- Python 3.10 or later
- Python virtual environment (optional but recommended)

### Install Requirements

Install package dependencies -- these are usually already installed if you are using `alsa` audio drivers.

```shell
sudo apt install libportaudio2
```

Install Python requirements for this example (ideally into your virtual environment):

```shell
pip install -r requirements.txt
```

## Run the Application

```shell
python app.py
```

### Options

- `--host`: Bind address for the web UI and websocket server (default: `0.0.0.0`)
- `--port`: Starting port (default: `8000`)
  - Web UI: `port`
  - WebSocket: `port + 1`
- `--telemetry`: Optional telemetry endpoint (default: disabled)

Once running, visit [http://localhost:8000](http://localhost:8000) in your browser.

## Reactor Components

Here's a high-level overview of the reactors implemented in this example:

- **`AudioBridge`**: Streams audio to the hardware device from frame blocks, with real-time underrun metrics.
- **`Synthesizer`**: Converts note-on/off events into deterministic, phase-aligned audio frames using periodic timers.
- **`WebsocketServer`**: Accepts incoming WebSocket connections and parses keyboard events or exit commands.
- **`Webserver`**: Serves static HTML/CSS/JS assets from a `static/` directory.
- **`LifecycleManager`**: Coordinates clean startup, shutdown, and inter-reactor stop signaling.

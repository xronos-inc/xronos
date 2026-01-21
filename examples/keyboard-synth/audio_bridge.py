# SPDX-FileCopyrightText: (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import os
import threading
from typing import Any, Callable

import numpy as np
import sounddevice  # type: ignore
import xronos
from numpy.typing import NDArray

from reactor_log import log

# pyright: standard


class AudioBridge(xronos.Reactor):
    """Audio bridge buffers raw audio data and streams to an audio device."""

    input = xronos.InputPortDeclaration[NDArray[np.float32]]()

    samplerate: int = 22050
    sampleperiod_s: float = 1.0 / samplerate
    channels: int = 1

    def __init__(self) -> None:
        super().__init__()

        self.__stream = sounddevice.OutputStream(
            samplerate=AudioBridge.samplerate,
            blocksize=0,  # let the driver choose the optimal blocksize
            channels=AudioBridge.channels,
            dtype=np.float32,
            latency="low",
            callback=self.__device_async_request,
            finished_callback=lambda: self.__stopped_callback.trigger(None),
        )

        # audio buffer
        self.__buffer: np.ndarray = np.empty(0, dtype=np.float32)
        self.__buffer_lock = threading.Lock()
        self.__buffer_underruns = 0  # buffer underruns since last block
        self.__stream_dac_t0 = datetime.datetime.fromtimestamp(0.0)

        # Audio streams often trigger buffer underruns at startup, causing the audio
        # driver to emit error messages directly to stderr. These low-level messages are
        # not catchable in Python and can clutter the console with non-actionable noise.
        # Since this reactor already reports underruns via metrics, the stderr output is
        # redundant. Unfortunately, the only way to suppress such messages is by
        # redirecting stderr for the entire process. As a workaround, stderr can
        # optionally be suppressed for a short time after startup to silence these
        # initial messages, without hiding future errors.
        self.__mask_stderr_duration_s: float | None = None
        self.__unmask_stderr = threading.Event()
        self.__mask_stderr_thread = threading.Thread(target=self.__mask_stderr)

        # reactor attributes
        self.add_attribute("subsystem", "audio")

    #########################
    # audio streaming bridge
    #########################

    __buffer_size_metric = xronos.MetricDeclaration(
        "audio buffer level", "bytes", {"buffer": "size"}
    )
    __buffer_underruns_metric = xronos.MetricDeclaration(
        "audio buffer underruns", "count", {"buffer": "underrun"}
    )

    @xronos.reaction
    def __on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        trigger = interface.add_trigger(self.input)
        buffer_size_metric = interface.add_effect(self.__buffer_size_metric)
        buffer_underruns_metric = interface.add_effect(self.__buffer_underruns_metric)

        def handler() -> None:
            buffer_len = 0
            buffer_underruns = 0

            # critical block: concatenate buffer and capture metrics
            with self.__buffer_lock:
                self.__buffer = np.concatenate(
                    (self.__buffer, trigger.get().astype(np.float32))
                )
                buffer_len = len(self.__buffer)
                buffer_underruns = self.__buffer_underruns
                self.__buffer_underruns = 0

            buffer_size_metric.record(buffer_len)
            buffer_underruns_metric.record(buffer_underruns)

        return handler

    # asynchronous callback from the audio device
    # this method executes in a thread owned by the device driver
    # it is called at high frequency to request the next block of audio frames
    def __device_async_request(  # type: ignore
        self,
        outdata: NDArray[np.float32],
        frames: int,
        time: Any,
        status: sounddevice.CallbackFlags,
    ) -> None:
        # pop frames available and write to device outdata
        frames_available = 0
        with self.__buffer_lock:
            frames_available = min(len(self.__buffer), frames)
            if frames_available > 0:
                outdata[:frames_available, 0] = self.__buffer[:frames_available]
                self.__buffer = self.__buffer[frames_available:]
                if frames_available < frames:
                    self.__buffer_underruns += 1

        # fill any remaining frames with zeros
        if frames_available < frames:
            outdata[frames_available:frames, 0] = 0.0

    def __mask_stderr(self) -> None:
        """Redirect stderr until an event is raised."""
        old_stderr = -1
        try:
            old_stderr = os.dup(2)
            devnull = os.open(os.devnull, os.O_WRONLY)
            os.dup2(devnull, 2)
            os.close(devnull)
            self.__unmask_stderr.wait(timeout=self.__mask_stderr_duration_s)
        except OSError:
            pass
        finally:
            if old_stderr >= 0:
                os.dup2(old_stderr, 2)
                os.close(old_stderr)

    ####################
    # service lifecycle
    ####################

    # program lifecycle management
    request_service_stop = xronos.InputPortDeclaration[None]()
    service_stopped = xronos.OutputPortDeclaration[None]()
    __stopped_callback = xronos.PhysicalEventDeclaration[None]()

    @xronos.reaction
    def __on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)

        def handler() -> None:
            log(self, "starting")
            self.__mask_stderr_thread.start()
            self.__stream.start()
            self.__stream_dac_t0 = datetime.datetime.fromtimestamp(self.__stream.time)

        return handler

    @xronos.reaction
    def __on_request_service_stop(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.request_service_stop)
        stopped_effect = interface.add_effect(self.service_stopped)

        def handler() -> None:
            if self.__stream.active:
                log(self, "stopping")
                self.__stream.stop()  # issues callback when stopped
            else:
                stopped_effect.set(None)

        return handler

    @xronos.reaction
    def __on_stopped_callback(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        interface.add_trigger(self.__stopped_callback)
        stopped_effect = interface.add_effect(self.service_stopped)

        def handler() -> None:
            log(self, "stopped")
            self.__unmask_stderr.set()
            stopped_effect.set(None)

        return handler

    @xronos.reaction
    def __on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            if self.__stream.active:
                log(self, "warning: service was not stopped before shutdown")
                self.__stream.abort()
            self.__unmask_stderr.set()
            self.__mask_stderr_thread.join()

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

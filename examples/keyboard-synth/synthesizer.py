# SPDX-FileCopyrightText: (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

import datetime
from typing import Callable, Dict, Set

import numpy as np
import xronos
from numpy.typing import NDArray

from keypress import Keypress
from reactor_log import log


class Synthesizer(xronos.Reactor):
    """Synthesizer generates an audio stream from keypress events.

    This is a Xronos reactor that generates audio frames based on
    real-time keypress events from multiple clients.

    === Input Ports ===

    - keypress: InputPort[Keypress]
        Receives keypress events per client. Each event specifies a note string
        (e.g., "A4") and whether the note is pressed or released.

    === Physical Events ===

    - remove_client: PhysicalEvent[int]
        Removes all active notes associated with a given client ID.

    === Output Ports ===

    - frames: OutputPort[NDArray[np.float32]]
        Emits audio blocks at regular intervals, corresponding to the
        currently active set of notes. If no notes are active, emits silence.

    === Audio Synthesis ===

    - Uses additive synthesis (sum of sine waves) for each active note.
    - Frequencies are mapped from a fixed note-to-frequency table.
    - Output is normalized to maintain constant RMS amplitude.
    """

    keypress = xronos.InputPortDeclaration[Keypress]()
    remove_client = xronos.PhysicalEventDeclaration[int]()

    frames = xronos.OutputPortDeclaration[NDArray[np.float32]]()
    __update_timer = xronos.PeriodicTimerDeclaration()

    def __init__(self, samplerate: int, blocksize: int) -> None:
        super().__init__()
        self._samplerate = samplerate
        self._blocksize = blocksize
        self._clients: Dict[int, Set[str]] = {}
        self.__update_timer.period = datetime.timedelta(
            seconds=self._blocksize / float(samplerate)
        )

    @staticmethod
    def format_notes(notes: Set[str]) -> str:
        """Create a human-readable list of active notes."""
        return " ".join(sorted(notes)) if notes else "nothing"

    @xronos.reaction
    def __on_keypress(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Update active client keys in response to a keypress event."""
        trigger = interface.add_trigger(self.keypress)

        def handler() -> None:
            # update client active keys
            kp = trigger.get()
            notes = self._clients.setdefault(kp.client_id, set())
            if kp.enabled:
                notes.add(kp.note)
            else:
                notes.discard(kp.note)
            log(self, f"playing {Synthesizer.format_notes(notes)}")

        return handler

    @xronos.reaction
    def __on_remove_client(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        trigger = interface.add_trigger(self.remove_client)

        def handler() -> None:
            self._clients.pop(trigger.get(), None)

        return handler

    @xronos.reaction
    def __on_update_timer(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Generate the next block of audio frames.

        Tones are generated using reactor time, and consequently
        are guarnteed to be phase aligned between blocks. This
        demonstates a deterministic generation of a time-based signal.
        """
        interface.add_trigger(self.__update_timer)
        effect = interface.add_effect(self.frames)

        def handler() -> None:
            t0 = self.get_time_since_startup().total_seconds()
            dt = 1.0 / self._samplerate
            t = np.arange(self._blocksize, dtype=np.float32) * dt + t0
            frequencies = self.get_frequencies()
            block: NDArray[np.float32]
            if frequencies:
                block = np.sum(
                    [np.sin(2 * np.pi * f * t) for f in frequencies], axis=0
                ).astype(np.float32)
                block /= np.sqrt(len(frequencies))  # ensure constant RMS power
            else:
                block = np.zeros_like(t, dtype=np.float32)
            effect.set(block)

        return handler

    def get_frequencies(self) -> Set[float]:
        note_to_freq: Dict[str, float] = {
            "C4": 261.63,
            "C#4": 277.18,
            "D4": 293.66,
            "D#4": 311.13,
            "E4": 329.63,
            "F4": 349.23,
            "F#4": 369.99,
            "G4": 392.00,
            "G#4": 415.30,
            "A4": 440.00,
            "A#4": 466.16,
            "B4": 493.88,
            "C5": 523.25,
        }
        frequencies: Set[float] = set()
        for client_notes in self._clients.values():
            for note in client_notes:
                frequencies.add(note_to_freq[note])
        return frequencies

    ##################
    # logging
    ##################

    # log from threads not managed by the xronos runtime
    __async_log = xronos.PhysicalEventDeclaration[str]()

    @xronos.reaction
    def __on_async_log(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        async_log = interface.add_trigger(self.__async_log)
        return lambda: log(self, async_log.get())

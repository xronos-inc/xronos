# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-FileCopyrightText: Copyright (c) 2022 The Lingua Franca Coordination Language.
# SPDX-License-Identifier: BSD-3-Clause
#
# This file contains portions from the Lingua Franca Coordination Language
# under BSD-2-Clause and modifications Xronos Inc. under BSD-3-Clause.

import datetime
import math
import sys
from typing import Callable

import cv2
import xronos


class WebCam(xronos.Reactor):
    _sample_webcam = xronos.ProgrammableTimerDeclaration[None]()
    frame = xronos.OutputPortDeclaration[cv2.typing.MatLike]()

    def __init__(
        self, frames_per_second: int, video_stream_path: str | None = None
    ) -> None:
        super().__init__()
        self.sample_period = datetime.timedelta(seconds=1.0 / frames_per_second)
        print(self.sample_period)

        if video_stream_path is not None:
            self.stream = cv2.VideoCapture(video_stream_path)
        else:
            self.stream = cv2.VideoCapture(0, cv2.CAP_ANY)

        if not self.stream.isOpened():
            print("ERROR: Could not open VideoCapture stream")
            sys.exit(1)

        self.stream.set(cv2.CAP_PROP_FPS, frames_per_second)

    def reschedule_sample_webcam(
        self, ctx: xronos.ReactionContext, action: xronos.ProgrammableTimerEffect[None]
    ) -> None:
        next_sample_time = self.sample_period + self.sample_period * math.floor(
            ctx.lag / self.sample_period
        )
        action.schedule(value=None, delay=next_sample_time)

    @xronos.reaction
    def on_startup(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)
        sample_webcam_effect = ctx.add_effect(self._sample_webcam)

        def handler() -> None:
            self.reschedule_sample_webcam(ctx, sample_webcam_effect)

        return handler

    @xronos.reaction
    def on_sample_webcam(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self._sample_webcam)
        sample_webcam_effect = ctx.add_effect(self._sample_webcam)
        frame_effect = ctx.add_effect(self.frame)
        shutdown_effect = ctx.add_effect(self.shutdown)

        def handler() -> None:
            ret, _frame = self.stream.read()
            if ret:
                frame_effect.set(_frame)
            else:
                print("WARNING: Failed to read frames from videostream. Shutting down.")
                shutdown_effect.trigger_shutdown()
            self.reschedule_sample_webcam(ctx, sample_webcam_effect)

        return handler

    @xronos.reaction
    def on_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.shutdown)

        def handler() -> None:
            print("Cleaning up WebCam")
            if self.stream.isOpened():
                self.stream.release()

        return handler

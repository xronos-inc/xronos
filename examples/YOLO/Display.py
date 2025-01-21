# SPDX-FileCopyrightText: (c) 2022 The Lingua Franca Coordination Language.
# SPDX-License-Identifier: BSD-2-Clause
#
# Modifications Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause
#
# This file contains portions from the Lingua Franca Coordination Language
# under BSD-2-Clause and modifications Xronos Inc. under BSD-3-Clause.

from typing import Callable

import cv2
import numpy as np
import xronos
from numpy.typing import NDArray

LABEL_COLOR = (255, 100, 0)


class Display(xronos.Reactor):
    # Display requires events frame and dnn_result to be simultaneous.
    frame = xronos.InputPortDeclaration[cv2.typing.MatLike]()
    dnn_result = xronos.InputPortDeclaration[
        tuple[list[str], NDArray[np.float32], NDArray[np.float32]]
    ]()

    def __init__(
        self, confidence_threshold: float = 0.7, no_display: bool = False
    ) -> None:
        super().__init__()
        self.confidence_threshold = confidence_threshold
        self.label_color = LABEL_COLOR
        self.no_display = no_display

    @xronos.reaction
    def on_frame(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        frame_trigger = interface.add_trigger(self.frame)
        dnn_result_trigger = interface.add_trigger(self.dnn_result)

        def handler() -> None:
            if not frame_trigger.is_present() or not dnn_result_trigger.is_present():
                print(
                    "ERROR: Display requires events frame and dnn_result simultaneously"
                )
                return

            current_frame = frame_trigger.get()
            current_dnn_result = dnn_result_trigger.get()

            x_shape, y_shape = current_frame.shape[1], current_frame.shape[0]
            # Draw all the object detection triangles and dnn_result
            for name, confidence, coordinate in zip(*current_dnn_result):
                if confidence > self.confidence_threshold:
                    x1, y1 = int(coordinate[0] * x_shape), int(coordinate[1] * y_shape)  # type: ignore[index]
                    x2, y2 = int(coordinate[2] * x_shape), int(coordinate[3] * y_shape)  # type: ignore[index]
                    cv2.rectangle(
                        current_frame, (x1, y1), (x2, y2), self.label_color, 2
                    )
                    cv2.putText(
                        current_frame,
                        name,
                        (x1, y1),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        self.label_color,
                        2,
                    )

            if not self.no_display:
                # Draw actual frame
                cv2.imshow("frame", current_frame)
                # Terminate on press-q
                cv2.putText(
                    current_frame,
                    "Exit with `q`",
                    (0, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 0),
                    2,
                )
                if cv2.waitKey(1) == ord("q"):
                    self.environment.request_shutdown()

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            print("Cleaning up Display")
            cv2.destroyAllWindows()

        return handler

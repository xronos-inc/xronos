# SPDX-FileCopyrightText: © 2022 The Lingua Franca Coordination Language.
# SPDX-License-Identifier: BSD-2-Clause
#
# Modifications Copyright © 2024 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause
#
# This file contains portions from the Lingua Franca Coordination Language
# under BSD-2-Clause and modifications Xronos Inc. under BSD-3-Clause.

from typing import Callable

import cv2
import numpy as np
import xronos

LABEL_COLOR = (255, 100, 0)


class Display(xronos.Reactor):
    # Display requires events frame and dnn_result to be simultaneous.
    frame = xronos.InputPortDeclaration[cv2.typing.MatLike]()
    dnn_result = xronos.InputPortDeclaration[
        tuple[np.ndarray, np.ndarray, np.ndarray]
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
        frame = interface.add_trigger(self.frame)
        dnn_result = interface.add_trigger(self.dnn_result)

        def handler() -> None:
            if not frame.is_present or not dnn_result.is_present:
                print(
                    "ERROR: Display requires events frame and dnn_result simultanously"
                )
            x_shape, y_shape = frame.value.shape[1], frame.value.shape[0]
            # Draw all the object detection triangles and dnn_result
            for name, confidence, coordinate in zip(*dnn_result.value):
                if confidence > self.confidence_threshold:
                    x1, y1 = int(coordinate[0] * x_shape), int(coordinate[1] * y_shape)
                    x2, y2 = int(coordinate[2] * x_shape), int(coordinate[3] * y_shape)
                    cv2.rectangle(frame.value, (x1, y1), (x2, y2), self.label_color, 2)
                    cv2.putText(
                        frame.value,
                        name,
                        (x1, y1),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        self.label_color,
                        2,
                    )

            if not self.no_display:
                # Draw actual frame
                cv2.imshow("frame", frame.value)
                # Terminate on press-q
                cv2.putText(
                    frame.value,
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

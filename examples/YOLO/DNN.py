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
import torch
import ultralytics  # type: ignore
import xronos


class DNN(xronos.Reactor):
    frame = xronos.InputPortDeclaration[cv2.typing.MatLike]()
    result = xronos.OutputPortDeclaration[tuple[np.ndarray, np.ndarray, np.ndarray]]()

    def __init__(self, model_path: str) -> None:
        super().__init__()
        if torch.cuda.is_available():
            print("Using CUDA for inference")
            self.device = "cuda"
        else:
            print("Using CPU for inference")
            self.device = "cpu"
        self.model: ultralytics.YOLO = ultralytics.YOLO(model_path, verbose=False)  # type: ignore

    @xronos.reaction
    def on_frame(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        frame = interface.add_trigger(self.frame)
        result = interface.add_effect(self.result)

        def handler() -> None:
            # Run the model on the frame
            results: ultralytics.engine.results.Results = self.model([frame.value])[0]  # type: ignore
            # Extract names, bounding boxes and confidences scores from the results.
            boundingBoxes = results.boxes.xyxyn.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            names = [results.names[x] for x in results.boxes.cls.cpu().numpy()]
            result.value = (
                np.array(names),
                np.array(confidences),
                np.array(boundingBoxes),
            )

        return handler
# SPDX-FileCopyrightText: (c) 2022 The Lingua Franca Coordination Language.
# SPDX-License-Identifier: BSD-2-Clause
#
# Modifications Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause
#
# This file contains portions from the Lingua Franca Coordination Language
# under BSD-2-Clause and modifications by Xronos Inc. under BSD-3-Clause.

import argparse
from typing import Optional

import xronos

from Display import Display
from DNN import DNN
from WebCam import WebCam


def main(
    video_path: Optional[str] = None,
    no_display: bool = False,
    telemetry: bool = False,
) -> None:
    env = xronos.Environment()
    webCam = env.create_reactor(
        "WebCam", WebCam, frames_per_second=10, video_stream_path=video_path
    )
    dnn = env.create_reactor("DNN", DNN, model_path="yolo11n.pt")
    display = env.create_reactor("Display", Display, no_display=no_display)

    env.connect(webCam.frame, display.frame)
    env.connect(webCam.frame, dnn.frame)
    env.connect(dnn.result, display.dnn_result)

    if telemetry:
        print("enable telemetry")
        env.enable_telemetry()

    env.execute()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A Xronos example using YOLOv11 for object detection."
    )
    parser.add_argument(
        "--video-path",
        type=str,
        help="Path to a video capture file. If not None, the default webcam is used.",
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Dont display the video stream, print values to stdout instead.",
    )
    parser.add_argument(
        "--telemetry",
        action="store_true",
        help="Enable the xronos telemetry feature.",
    )
    args = parser.parse_args()
    main(args.video_path, args.no_display, args.telemetry)

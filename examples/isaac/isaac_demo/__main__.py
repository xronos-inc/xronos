# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from .demo import demo

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A Xronos example for interfacing with Isaac Sim."
    )
    parser.add_argument(
        "--telemetry",
        action="store_true",
        help="Send telemetry data to the Xronos dashboard.",
    )
    args = parser.parse_args()

    demo(args.telemetry)

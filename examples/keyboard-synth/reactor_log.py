# SPDX-FileCopyrightText: (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Standardized log message for any reactor."""

# pyright: standard

import xronos


def log(name: str, ctx: xronos.ReactionContext, msg: str) -> None:
    """Print a log message to the console with reactor name and current time."""
    print(f"[{ctx.current_time:%H:%M:%S.%f}] ({name}) {msg}")

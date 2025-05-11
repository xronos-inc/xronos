# SPDX-FileCopyrightText: (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Standardized log message for any reactor."""

# pyright: standard

import xronos


def log(reactor: xronos.Reactor, msg: str) -> None:
    """Print a log message to the console with reactor name and current time."""
    total_us = int(reactor.get_time_since_startup().total_seconds() * 1_000_000)
    seconds = total_us // 1_000_000
    microseconds = total_us % 1_000_000
    milliseconds = microseconds // 1_000_000
    micros_remainder = microseconds % 1_000_000
    print(
        f"[{seconds:03}:{milliseconds:03}.{micros_remainder:06}] ({reactor.name}) {msg}"
    )

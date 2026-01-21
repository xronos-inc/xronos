# SPDX-FileCopyrightText: (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

from dataclasses import dataclass


@dataclass
class Keypress:
    """Keypress event representing a down/up press of a synthesizer key."""

    client_id: int
    note: str
    enabled: bool

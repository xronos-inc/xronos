# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

"""Library reactors: reactors the SDK ships ready for use.

The external-input reactors read input from outside the application. The
others produce outputs such as constants, successive values, and ramps, at
startup, from a timer, or when triggered.
"""

from xronos.lib._external_input import ConsoleInput, ExternalInput
from xronos.lib._simple_source import (
    ConstSource,
    OutputDiscardedWarning,
    RampSource,
    StartupSource,
    SuccessorSource,
    TimerSource,
)
from xronos.lib._socket_input import SocketInput

__all__ = [
    "ConsoleInput",
    "ConstSource",
    "ExternalInput",
    "OutputDiscardedWarning",
    "RampSource",
    "SocketInput",
    "StartupSource",
    "SuccessorSource",
    "TimerSource",
]

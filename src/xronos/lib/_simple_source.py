# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import copy
import datetime
from typing import (
    Callable,
    Optional,
    TypeAlias,
    TypeVar,
    Union,
    cast,
    final,
)

# 'override' added to typing in 3.12
from typing_extensions import override

import xronos
from xronos.lib import _abstract_source as abstract_source

OutputDiscardedWarning: TypeAlias = abstract_source.OutputDiscardedWarning

TOutput = TypeVar("TOutput")


class TimerSource(abstract_source.AbstractTimerSource[None]):
    """Simple reactor that produces a pure (valueless) event from a periodic timer.

    Args:
        period: The interval at which the timer event is triggered.
        inhibit: Initialize with output inhibited.
    """

    def __init__(
        self, period: Optional[datetime.timedelta] = None, inhibit: bool = False
    ) -> None:
        super().__init__(period=period, inhibit=inhibit)

    @final
    @override
    def _timer_handler(self) -> None:
        return None


class StartupSource(abstract_source.AbstractSource[TOutput]):
    """A reactor that produces a single output at startup.

    Args:
        value: The value to output at startup
    """

    def __init__(self, value: TOutput) -> None:
        super().__init__()
        self._value: TOutput = value

    @override
    def _startup_handler(self) -> TOutput:
        return self._value


class ConstSource(abstract_source.AbstractTriggeredSource[TOutput]):
    """A reactor that produces a constant value when triggered.

    Args:
        value: The value to output at each triggered event.
        inhibit: Initialize with output inhibited.
    """

    set_value_port = xronos.InputPortDeclaration[TOutput]()

    def __init__(
        self,
        value: TOutput,
        inhibit: bool = False,
    ) -> None:
        super().__init__(inhibit=inhibit)
        self.__value: TOutput = value

    @override
    def _trigger_handler(self) -> TOutput:
        return self.__value

    @final
    @xronos.reaction
    def _on_value(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the reaction to the new value port."""
        trigger = interface.add_trigger(self.set_value_port)

        def handler() -> None:
            self.__value = copy.deepcopy(trigger.get())

        return handler


class SuccessorSource(abstract_source.AbstractTriggeredSource[TOutput]):
    """A reactor that produces a successive output when triggered.

    Custom successor behavior may be implemented by providing a successor function
    `successor(TOutput) -> TOutput`. This method should be stateless, reentrant
    and artifact-free to ensure thread-safety.

    Args:
        initial_value: The initial value of the source.
        successor: Successor operator for the type.
        inhibit: Initialize with output inhibited.
    """

    reset = xronos.InputPortDeclaration[None]()

    def __init__(
        self,
        initial_value: TOutput,
        successor: Callable[[TOutput], TOutput],
        inhibit: bool = False,
    ) -> None:
        super().__init__(inhibit=inhibit)

        # type may be a reference, so perform a deep copy
        self.__initial_value: TOutput = copy.deepcopy(initial_value)
        self.__value: TOutput = self.__initial_value
        self.__successor: Callable[[TOutput], TOutput] = successor
        # number of times an output has been produced since last reset
        self.__count: int = 0

    @final
    def count(self) -> int:
        """Number of times an output has been produced.

        This count is reset if the reset port is triggered.
        """
        return self.__count

    @override
    @final
    def _trigger_handler(self) -> TOutput:
        if self.__count > 0:
            # initial value has been output, so execute successor
            self.__value = self.__successor(self.__value)
        self.__count = self.__count + 1
        return self.__value

    @final
    @xronos.reaction
    def _on_reset(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.reset)

        def handler() -> None:
            self.__value = self.__initial_value
            self.__count = 0

        return handler


TRealOutput = TypeVar("TRealOutput", bound=Union[int, float])


class RampSource(SuccessorSource[TRealOutput]):
    """A reactor that produces a ramp output when triggered.

    If no arguments are provided, this source will produce an integer ramp starting at 0
    and incrementing by 1 at each trigger.

    Custom ramp behavior may be implemented by providing the successor function
    `successor(TRealOutput) -> TRealOutput`. This method should be stateless and
    reentrant to ensure thread safety.

    Args:
        initial_value: The value to output at each timer event.
        successor: Successor operator for the type.
        inhibit: Initialize with output inhibited.
    """

    def __init__(
        self,
        initial_value: TRealOutput = cast(TRealOutput, 0),
        successor: Optional[Callable[[TRealOutput], TRealOutput]] = None,
        inhibit: bool = False,
    ) -> None:
        super().__init__(
            initial_value=initial_value,
            successor=(
                successor if successor is not None else self.__default_successor
            ),
            inhibit=inhibit,
        )

    @staticmethod
    def __default_successor(x: TRealOutput) -> TRealOutput:
        return cast(TRealOutput, x + 1)

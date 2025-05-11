# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import warnings
from abc import abstractmethod
from typing import (
    Any,
    Callable,
    Generic,
    Optional,
    Type,
    TypeVar,
    final,
)

import xronos
from xronos._core import GenericEventSource

TOutput = TypeVar("TOutput")


class OutputDiscardedWarning(UserWarning):
    """Warning that output was discarded on an output port.

    Args:
        message: the warning message.
        fqn: fully qualified name of the reactor element that produced the output.
        value: value that was discarded.
        timestamp: the timestamp at which the value was discarded.
    """

    def __init__(
        self,
        message: Optional[str],
        fqn: Optional[str],
        value: Optional[Any],
        timestamp: Optional[datetime.datetime],
    ) -> None:
        self.custom_message = message
        self.fqn = fqn
        self.value = value
        self.timestamp = timestamp

        parts = [
            self.custom_message if self.custom_message else None,
            "Output",
            f"from {{{self.fqn}}}" if self.fqn else None,
            f"value: {{{self.value}}}" if self.value else None,
            "discarded",
            f"at timestamp {self.timestamp}." if self.timestamp else ".",
        ]
        result = " ".join(filter(None, parts))
        super().__init__(result)


class AbstractSource(xronos.Reactor, Generic[TOutput]):
    """Abstract reactor that is a simple source that produces a user-defined output.

    Derive from this class and override any of the following methods to produce output.
    - `_startup_handler`
    - `_shutdown_handler`

    Output from this reactor may be inhibited at startup or runtime. Inhibit changes
    are applied before any outputs are produced. If output is inhibited and a trigger
    is received, the appropriate handler is called, but its output is not produced.

    In case of simultaneous events, handlers will be executed in the order above,
    and the value from the last handler to execute will be output. A warning will
    be logged if the value from a previous handler was discarded.

    Args:
        inhibit: Initialize with output inhibited.
    """

    output = xronos.OutputPortDeclaration[TOutput]()
    inhibit = xronos.InputPortDeclaration[bool]()

    def __init__(
        self,
        inhibit: bool = False,
    ) -> None:
        super().__init__()
        self.__inhibit = inhibit

        self.warn_on_simultaneous_output = True

    @abstractmethod
    def _startup_handler(self) -> TOutput:
        """Handler for the startup reaction.

        Override this method to implement custom behavior.

        Raises:
            NotImplementedError if this abstract method is called.
        """
        raise NotImplementedError("Attempted to call an abstract reaction handler.")

    @abstractmethod
    def _shutdown_handler(self) -> TOutput:
        """Handler for the shutdown reaction.

        Override this method to implement custom behavior.

        Raises:
            NotImplementedError if this abstract method is called.
        """
        raise NotImplementedError("Attempted to call an abstract reaction handler.")

    TEventSource = TypeVar("TEventSource")

    @final
    def _create_reaction(
        self,
        interface: xronos.ReactionInterface,
        trigger: GenericEventSource[TEventSource],
        handler_base_class: Type["AbstractSource[TOutput]"],
        handler: Callable[..., TOutput],
    ) -> Callable[[], None]:
        """Conditionally specify a reaction with a single trigger and output.

        If `handler` is implemented in a subclass of `handler_base_class`, the
        reaction is specified with a trigger and the result of `handler()` is
        passed to the output port as an effect.

        If `handler` is abstract, the reaction has no body, triggers or effects.

        Args:
            interface: Reaction interface to add trigger and effect.
            trigger: The trigger for the reaction.
            handler_base_class: The base class for the abstract handler.
            handler: The handler to call in the reaction body. Must be bound method.

        Returns:
            Reaction handler.

        Raises:
            ValueError if `handler` is not an instance method of this object.
        """
        if not callable(getattr(self.__class__, handler.__name__, None)):
            raise ValueError(f"Handler must be an instance method of {self.__class__}")
        if not self.__is_method_overridden(handler_base_class, handler):
            return lambda: None

        interface.add_trigger(trigger)
        output_effect = interface.add_effect(self.output)

        def body() -> None:
            value = handler()

            if self.__inhibit:
                return

            if output_effect.is_present() and self.warn_on_simultaneous_output:
                warnings.warn(
                    OutputDiscardedWarning(
                        message="Simultaneous writes to an output port occurred.",
                        fqn=self.output.fqn,
                        value=output_effect.get(),
                        timestamp=self.get_time(),
                    )
                )
            output_effect.set(value)

        return body

    @final
    @xronos.reaction
    def _on_inhibit(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the reaction to the inhibit port.

        Sets the inhibit state of this reactor.

        Returns:
            Inhibit reaction handler.
        """
        inhibit_trigger = interface.add_trigger(self.inhibit)

        def handler() -> None:
            self.__inhibit = inhibit_trigger.get()

        return handler

    @final
    @xronos.reaction
    def _on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the startup reaction to invoke the `_startup_handler` method.

        Returns:
            Startup reaction handler.
        """
        return self._create_reaction(
            interface,
            self.startup,
            AbstractSource[TOutput],  # type: ignore[type-abstract]
            self._startup_handler,
        )

    @final
    @xronos.reaction
    def _on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the Shutdown reaction to invoke the `_shutdown_handler` method.

        Returns:
            Shutdown reaction handler.
        """
        return self._create_reaction(
            interface,
            self.shutdown,
            AbstractSource[TOutput],  # type: ignore[type-abstract]
            self._shutdown_handler,
        )

    def __is_method_overridden(
        self,
        base_class: Type["AbstractSource[TOutput]"],
        method: Callable[..., TOutput],
    ) -> bool:
        """Determine if a given method is overridden in a subclass of a parent class.

        Args:
            base_class: The base class where the method is defined.
            method: The method to check.
                Must be bound to an instance of the parent class or its subclass.

        Raises:
            ValueError if `method` is not an instance method.
        """
        if not callable(method):
            raise ValueError("Method must be callable")
        if not callable(getattr(base_class, method.__name__, None)):
            raise ValueError(
                f"Method must be an instance method of {base_class.__name__}"
            )

        method_name: str = method.__name__

        # get the method from the base class and the current class
        base_method = getattr(base_class, method_name, None)
        current_method = getattr(self.__class__, method_name, None)

        if current_method is None or base_method is None:
            return False
        return current_method != base_method


class AbstractTriggeredSource(AbstractSource[TOutput]):
    """Abstract reactor that produces output when triggered.

    Derive from this class and override any of the following methods to produce output:
    - `_startup_handler`
    - `_trigger_handler`
    - `_shutdown_handler`

    Output from this reactor may be inhibited at startup or runtime. Inhibit changes
    are applied before any outputs are produced. If output is inhibited and a trigger
    is received, the appropriate handler is called, but its output is not produced.

    In case of simultaneous events, handlers will be executed in the order above,
    and the value from the last handler to execute will be output. A warning will
    be logged if the value from a previous handler was discarded.
    """

    trigger = xronos.InputPortDeclaration[None]()

    @abstractmethod
    def _trigger_handler(self) -> TOutput:
        """Handler for the triggered reaction.

        Override this method to implement custom behavior.

        Raises:
            NotImplementedError if this abstract method is called.
        """
        raise NotImplementedError("Attempted to call an abstract reaction handler.")

    @final
    @xronos.reaction
    def _on_trigger(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the triggered reaction to invoke the `_trigger_handler` method.

        Returns:
            Input trigger reaction handler.
        """
        return self._create_reaction(
            interface,
            self.trigger,
            AbstractTriggeredSource[TOutput],  # type: ignore[type-abstract]
            self._trigger_handler,
        )


class AbstractTimerSource(AbstractSource[TOutput]):
    """Abstract reactor that produces output in response to timed triggers.

    Derive from this class and override any of the following methods to produce output:
    - `_startup_handler`
    - `_timer_handler`
    - `_shutdown_handler`

    Output from this reactor may be inhibited at startup or runtime. Inhibit changes
    are applied before any outputs are produced. If output is inhibited and a trigger
    is received, the appropriate handler is called, but its output is not produced.

    In case of simultaneous events, handlers will be executed in the order above,
    and the value from the last handler to execute will be output. A warning will
    be logged if the value from a previous handler was discarded.

    Args:
        period: The interval at which the timer event is triggered.
        inhibit: Initialize with output inhibited.
    """

    timer = xronos.PeriodicTimerDeclaration()

    def __init__(
        self,
        period: Optional[datetime.timedelta] = None,
        offset: Optional[datetime.timedelta] = None,
        inhibit: bool = False,
    ) -> None:
        super().__init__(inhibit)
        if period is not None:
            self.timer.period = period
        if offset is not None:
            self.timer.offset = offset

    @abstractmethod
    def _timer_handler(self) -> TOutput:
        """Handler for the timer reaction.

        Override this method to implement custom behavior.

        Raises:
            NotImplementedError if this abstract method is called.
        """
        raise NotImplementedError("Attempted to call an abstract reaction handler.")

    @final
    @xronos.reaction
    def _on_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Specify the timer reaction to invoke the `_timer_handler` method.

        Returns:
            Timer reaction handler.
        """
        return self._create_reaction(
            interface,
            self.timer,
            AbstractTimerSource[TOutput],  # type: ignore[type-abstract]
            self._timer_handler,
        )

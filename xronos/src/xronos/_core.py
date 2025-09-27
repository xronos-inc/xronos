# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# Ignore access to protected/private members. We use access to protected
# members of other classes to implement accessor objects.
# pyright: reportPrivateUsage = false
# Ignore missing source for xronos._runtime
# pyright: reportMissingModuleSource = false

import datetime
import inspect
import os
import sys
from typing import (
    Any,
    Callable,
    Generic,
    ParamSpec,
    Type,
    TypeAlias,
    TypeVar,
    cast,
    get_origin,
    overload,
)

import xronos._cpp_sdk as sdk

# Self is only available for Python>=3.11
if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

T = TypeVar("T")
R = TypeVar("R", bound="Reactor")
P = ParamSpec("P")

SdkElem = TypeVar("SdkElem", bound=sdk.Element)
Elem = TypeVar("Elem", bound="Element")


AttributeValue: TypeAlias = str | bool | int | float
AttributeMap: TypeAlias = dict[str, AttributeValue]


def get_source_location() -> sdk.SourceLocation:
    stack = inspect.stack()
    # The frame that we are looking for is 2 frames up the stack. 1 up gets to
    # the caller of this function and 2 up gets to their caller.
    frame = stack[2]
    # In case of generics there might be another level of indirection and we
    # have to go one more frame up.
    if frame.filename.endswith("typing.py"):
        frame = stack[3]

    try:
        pos = frame.positions
    except AttributeError:
        # Fallback for Python 3.10
        pos = None
    lineno = pos.lineno if pos and pos.lineno else frame.lineno
    end_lineno = pos.end_lineno if pos and pos.end_lineno else lineno
    col_offset = pos.col_offset if pos and pos.col_offset else 0
    end_col_offset = pos.end_col_offset if pos and pos.end_col_offset else 80
    return sdk.SourceLocation(
        function=frame.function,
        file_=frame.filename,
        start_line=lineno,
        end_line=end_lineno,
        start_column=col_offset,
        end_column=end_col_offset,
    )


class Element:
    """A reactor element.

    Reactor elements are objects that can be contained by reactors and that have
    special meaning to the Xronos SDK.

    Reactor elements should not be instantiated directly. Use the corresponding
    ``<Element>Declaration`` classes instead.
    """

    def __init__(self, sdk_instance: sdk.Element) -> None:
        self.__sdk_instance = sdk_instance

    def _get_sdk_instance(self, cls: Type[SdkElem]) -> SdkElem:
        assert isinstance(self.__sdk_instance, cls)
        return self.__sdk_instance

    @property
    def name(self) -> str:
        """Name of the element (read-only)."""
        return self.__sdk_instance.name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the element (read-only).

        The fully qualified name (FQN) represents the containment hierarchy. It
        consists of the containing reactor's FQN plus the element's name
        separated by a ``.``. For top-level reactors (those owned by the
        environment), the FQN is equal to the name.
        """
        return self.__sdk_instance.fqn

    def add_attribute(self, key: str, value: str | bool | int | float) -> None:
        """Annotate the element with an attribute.

        Adding the attribute only succeeds, if the given key has not been set
        before on the same element.

        See :ref:`attributes` for more information.

        Args:
            key: The name of the attribute to add
            value: The value of the attribute to add

        Raises:
            KeyError: If the attribute key was added before.
        """
        result = self.__sdk_instance.add_attribute(key, value)
        if not result:
            raise KeyError("Overwriting an existing attribute is not permitted.")

    def add_attributes(self, attributes: dict[str, str | bool | int | float]) -> None:
        """Annotate the element with multiple attributes.

        Adding the attributes only succeeds, if the given key has not been set
        before on the same element.

        See :ref:`attributes` for more information.

        Args:
            attributes: A map of attribute names and their values to be added.

        Raises:
            KeyError: If the attribute key was added before.
        """
        result = self.__sdk_instance.add_attributes(attributes)
        if not result:
            raise KeyError("Overwriting an existing attribute is not permitted.")


class ElementDescriptor(Generic[Elem]):
    """A descriptor for a reactor element.

    Args:
        initializer: a callable that can be used to create the element
        attributes: dictionary of attributes to add to the element
    """

    def __init__(
        self,
        initializer: Callable[[str, "Reactor"], Elem],
        attributes: AttributeMap | None = None,
    ) -> None:
        self.__name: str | None = None
        self.__initializer = initializer
        self.__attributes = attributes

    def __set_name__(self, reactor_cls: Type["Reactor"], name: str) -> None:
        """Record the element name and register the descriptor with the reactor class.

        This is called implicitly when assigning an instance of
        `ElementDescriptor` to a class attribute.
        """
        assert not self.__name
        self.__name = name
        reactor_cls._register_element_descriptor(self)

    @overload
    def __get__(self, reactor: "Reactor", _: type) -> Elem: ...
    @overload
    def __get__(self, reactor: None, _: type) -> Self: ...

    def __get__(self, reactor: "Reactor | None", _: type) -> Elem | Self:
        """Return the described element.

        Returns:
            The element when called on an object or `self` when called on a class.
        """
        if not reactor:
            return self
        return cast(Elem, reactor._get_element(self._name))

    @property
    def _name(self) -> str:
        assert self.__name
        return self.__name

    def _create_instance(self, reactor: "Reactor") -> Elem:
        """Return an instance of the described element.

        Args:
            reactor: the owning `Reactor`
        """
        instance = self.__initializer(self._name, reactor)
        if self.__attributes:
            instance.add_attributes(self.__attributes)
        return instance


class EventSource(Element, Generic[T]):
    """An element that may emit events and act as reaction trigger."""

    pass


class Startup(EventSource[None]):
    """A reactor element that emits an event program starts executing.

    Can be used as a reaction :class:`Trigger`.
    """

    pass


class Shutdown(EventSource[None]):
    """A reactor element that emits an event right before the program shuts down.

    Can be used as a reaction :class:`Trigger`.
    """

    pass


class PeriodicTimer(EventSource[None]):
    """A reactor element that emits events in regular intervals.

    Can be used as a reaction :class:`Trigger`.

    This class is not intended to be instantiated directly. Use
    :class:`PeriodicTimerDeclaration` instead.
    """

    @property
    def period(self) -> datetime.timedelta:
        """The delay in between two events emitted by the timer (read-write).

        Raises:
            RuntimeError: When set during program execution.
        """
        return self.__sdk_instance.period()

    @period.setter
    def period(self, value: datetime.timedelta) -> None:
        self.__sdk_instance.set_period(value)

    @property
    def offset(self) -> datetime.timedelta:
        """The delay between :attr:`~Reactor.startup` and the first event emitted by the timer (read-write).

        Raises:
            RuntimeError: When set during program execution.
        """  # noqa: E501
        return self.__sdk_instance.offset()

    @offset.setter
    def offset(self, value: datetime.timedelta) -> None:
        self.__sdk_instance.set_offset(value)

    @property
    def __sdk_instance(self) -> sdk.PeriodicTimer:
        return self._get_sdk_instance(sdk.PeriodicTimer)


class PeriodicTimerDeclaration(ElementDescriptor[PeriodicTimer]):
    """A declaration for a :class:`PeriodicTimer`.

    Args:
        period: The delay in between two events emitted by the timer (optional).
        offset: The delay between the :attr:`~Reactor.startup` event
            and the first event emitted by the timer (optional).
        attributes(optional): A dict of attributes characterizing the timer.
    """

    def __init__(
        self,
        period: datetime.timedelta = datetime.timedelta(0),
        offset: datetime.timedelta = datetime.timedelta(0),
        attributes: AttributeMap | None = None,
    ) -> None:
        source_location = get_source_location()

        def initialize(name: str, reactor: "Reactor") -> PeriodicTimer:
            context = reactor._context(source_location)
            sdk_timer = sdk.PeriodicTimer(name, context, period, offset)
            return PeriodicTimer(sdk_timer)

        super().__init__(initialize, attributes)


class InputPort(EventSource[T]):
    """A reactor element for receiving messages from other reactors.

    Input ports can be used as a reaction :class:`Trigger`
    and provide an interface for reactors to receive messages from other
    reactors.

    Input ports may be connected to other ports so that messages are forwarded
    automatically (see :func:`Environment.connect` and :func:`Reactor.connect`).

    Reactions of other reactors may also use input ports as a
    :class:`PortEffect` allowing an external reaction handler to send messages
    directly to the port.

    This class is not intended to be instantiated directly. Use
    :class:`InputPortDeclaration` instead.

    Type Parameters:
        T: The value type associated with messages.
    """

    pass


class InputPortDeclaration(ElementDescriptor[InputPort[T]]):
    """A declaration for an :class:`InputPort[T]<InputPort>`.

    Args:
        attributes(optional): A dict of attributes characterizing the input port.

    Type Parameters:
        T: The value type associated with messages.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        source_location = get_source_location()

        def initialize(name: str, reactor: "Reactor") -> InputPort[T]:
            context = reactor._context(source_location)
            return InputPort(sdk.InputPort(name, context))

        super().__init__(initialize, attributes)


class OutputPort(EventSource[T]):
    """A reactor element for sending messages to other reactors.

    Output ports can be used as a :class:`PortEffect` and provide an interface
    for reactors to send messages to other reactors.

    Output ports may be connected to other ports so that messages are forwarded
    automatically (see :func:`Environment.connect` and :func:`Reactor.connect`).

    Other reactors may also use output ports as a reaction :class:`Trigger`
    allowing an external reaction handler to receive messages directly from the
    port.

    This class is not intended to be instantiated directly. Use
    :class:`OutputPortDeclaration` instead.

    Type Parameters:
        T: The value type associated with messages.
    """

    pass


class OutputPortDeclaration(ElementDescriptor[OutputPort[T]]):
    """A declaration for an :class:`OutputPort[T]<OutputPort>`.

    Args:
        attributes(optional): A dict of attributes characterizing the output port.

    Type Parameters:
        T: The type of values carried by the port.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        source_location = get_source_location()

        def initialize(name: str, reactor: "Reactor") -> OutputPort[T]:
            context = reactor._context(source_location)
            return OutputPort(sdk.OutputPort(name, context))

        super().__init__(initialize, attributes)


class ProgrammableTimer(EventSource[T]):
    """A reactor element for scheduling new events.

    Programmable timers may be used by reactions to schedule new events that
    will be emitted in the future. They can be used both as a reaction
    :class:`Trigger` as and as a :class:`ProgrammableTimerEffect`.

    Type Parameters:
        T: The value type associated with events emitted by the programmable
           timer.
    """

    pass


class ProgrammableTimerDeclaration(ElementDescriptor[ProgrammableTimer[T]]):
    """A declaration for a :class:`ProgrammableTimer[T]<ProgrammableTimer>`.

    Args:
        attributes(optional): A dict of attributes characterizing the
        programmable timer.

    Type Parameters:
        T: The value type associated with events emitted by the programmable
           timer.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        source_location = get_source_location()

        def initialize(name: str, reactor: "Reactor") -> ProgrammableTimer[T]:
            context = reactor._context(source_location)
            return ProgrammableTimer(sdk.ProgrammableTimer(name, context))

        super().__init__(initialize, attributes)


class PhysicalEvent(EventSource[T]):
    """A reactor element for receiving events from external sources.

    Physical events may be used to trigger reactions from a context outside of
    the scope of the reactor program. These could be external event handlers that
    respond to sensor inputs.

    Can be used as a reaction :class:`Trigger` allowing the reaction handler to
    read the associated value.

    Type Parameters:
        T: The type of values carried by emitted events.
    """

    def trigger(self, value: T) -> None:
        """Emit a new event with an associated value and trigger reactions.

        The event will be assigned a timestamp equal to the current wall-clock
        time.

        Args:
            value: The value to be associated with the emitted event.
        """
        self._get_sdk_instance(sdk.PhysicalEvent).trigger(value)


class PhysicalEventDeclaration(ElementDescriptor[PhysicalEvent[T]]):
    """A declaration for a :class:`PhysicalEvent[T]<PhysicalEvent>`.

    Args:
        attributes(optional): A dict of attributes characterizing the physical event.

    Type Parameters:
        T: The type of values carried by emitted events.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        source_location = get_source_location()

        def initialize(name: str, reactor: "Reactor") -> PhysicalEvent[T]:
            context = reactor._context(source_location)
            return PhysicalEvent(sdk.PhysicalEvent(name, context))

        super().__init__(initialize, attributes)


class Metric(Element):
    """A reactor element for recording metric data to an external data base.

    Can be used by a reaction as a :class:`MetricEffect` allowing the
    reaction handler to record values using the metric.

    This class is not intended to be instantiated directly. Use
    :class:`MetricDeclaration` instead.
    """

    @property
    def description(self) -> str:
        """Description of the metric (read-only)."""
        return self.__sdk_instance.description

    @property
    def unit(self) -> str:
        """Unit of the metric (read-only)."""
        return self.__sdk_instance.unit

    @property
    def __sdk_instance(self) -> sdk.Metric:
        return self._get_sdk_instance(sdk.Metric)


class MetricDeclaration(ElementDescriptor[Metric]):
    """A declaration for a :class:`Metric`.

    Args:
        description: A description of the metric.
        unit: The unit of the metric.
        attributes: A dict of attributes characterizing the metric.
    """

    def __init__(
        self,
        description: str,
        unit: str | None = None,
        attributes: AttributeMap | None = None,
    ) -> None:
        if unit is None:
            unit = ""

        source_location = get_source_location()

        def initialize(name: str, reactor: "Reactor") -> Metric:
            context = reactor._context(source_location)
            return Metric(sdk.Metric(name, context, description, unit))

        super().__init__(initialize, attributes)


class Environment:
    """The entry point for assembling and executing reactor programs.

    The environment acts as an execution context for reactor programs. It
    manages both the creation of reactors and the execution of reactor
    programs.

    Args:
        fast: If set to `True`, enables a special mode of execution that skips
            waiting between executing events and instead processes events as
            fast as possible. This is relevant for testing. (optional)
        timeout: Specifies a maximum duration for which the program executes.
            When the program reaches the timeout it terminates and triggers
            :attr:`~xronos.Reactor.shutdown`. This is mostly useful for
            testing. (optional)
    """

    def __init__(
        self, fast: bool = False, timeout: datetime.timedelta | None = None
    ) -> None:
        workers = 1

        try:
            # This is currently the official way to check whether the GIL is
            # enabled or not. The function is only available in Python>=3.13.
            # For older versions, we simply catch the AttributeError and
            # continue.
            # see https://docs.python.org/3/library/sys.html#sys._is_gil_enabled
            if not sys._is_gil_enabled():  # type: ignore
                workers = cast(int, os.cpu_count())
        except AttributeError:
            pass

        if timeout:
            self.__sdk_env = sdk.Environment(
                workers, fast, render_reactor_graph=True, timeout=timeout
            )
        else:
            self.__sdk_env = sdk.Environment(workers, fast, render_reactor_graph=True)

        #: List of contained reactors.
        #:
        #: Used for keeping references to all reactors created via
        #: `create_reactor`. This ensures that the garbage collector does not
        #: delete reactors, even if the user does not assign them to a variable or
        #: the variable goes out of scope.
        self.__reactors = list["Reactor"]()

    @overload
    def connect(
        self,
        from_: OutputPort[T],
        to: InputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None: ...
    @overload
    def connect(
        self,
        from_: OutputPort[T],
        to: OutputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None: ...
    @overload
    def connect(
        self,
        from_: InputPort[T],
        to: InputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None: ...

    def connect(
        self,
        from_: InputPort[T] | OutputPort[T],
        to: InputPort[T] | OutputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None:
        """Connect two ports.

        Creates a new connection from the port given in ``from_`` to the port
        given in ``to``.

        If ``delay`` is ``None`` messages are delivered without a delay.
        This means that the timestamp at which the message is received is the
        same as the timestamp at which it was sent.

        If ``delay`` is set messages are delivered with a delay. This means
        that the timestamp at which the message is received is the timestamp at
        which it was sent plus delay.

        Args:
            from_(InputPort[T] | OutputPort[T]): The port to draw the
                connection from.
            to(InputPort[T] | OutputPort[T]): The port to draw the connection
                to.
            delay(~datetime.timedelta | None): The delay to apply to messages.
                (optional)

        Raises:
            ValidationError: If an invalid connections is created.
        """
        from_port = from_._get_sdk_instance(sdk.Element)
        to_port = to._get_sdk_instance(sdk.Element)
        assert isinstance(from_port, sdk.InputPort | sdk.OutputPort)
        assert isinstance(to_port, sdk.InputPort | sdk.OutputPort)
        if delay is None:
            self.__sdk_env.connect(from_port, to_port)
        else:
            self.__sdk_env.connect_delayed(from_port, to_port, delay)

    def create_reactor(
        self,
        name: str,
        reactor_class: Callable[P, R],
        *args: P.args,
        **kwargs: P.kwargs,
    ) -> R:
        """Create a new reactor.

        A factory method for instantiating and registering a new reactor.

        Args:
            name: Name of the reactor to be instantiated.
            reactor_class(type): The reactor class to be instantiated. Must be
               a subclass of :class:`~xronos.Reactor`.
            args(~typing.Any): Arguments to be passed to the ``__init__``
                method of ``reactor_class``.
            kwargs(~typing.Any): Keyword arguments to be passed to the
                ``__init__`` method of ``reactor_class``.

        Returns:
            ``reactor_class``:
                 An instance of the class provided in the ``reactor_class``
                 argument.
        """
        # We annotate `reactor_class` as a callable that returns an R to infer
        # the parameters of the init method. `reactor_class` is expected to be
        # a concrete reactor class. The Callable is needed to correctly infer
        # the types of all args and kwargs and support type checking any calls
        # to this method.
        checked_class = Reactor._checked_cast_to_subclass(reactor_class)
        context = self.__sdk_env.context(get_source_location())
        reactor = checked_class._create_instance(name, context, *args, **kwargs)

        # keep a reference to the created reactor
        self.__reactors.append(reactor)
        return reactor

    def execute(self) -> None:
        """Execute the reactor program.

        Initiates the execution of a reactor program. This triggers
        :attr:`~xronos.Reactor.startup` and instructs the runtime to start
        processing reactions.

        Returns when the reactor program terminates. The reactor program
        terminates when there are no more events, or after calling
        :func:`request_shutdown()`.

        Raises:
            ValidationError: When the program is invalid or reaches an invalid state.
        """
        return self.__sdk_env.execute()

    def enable_telemetry(
        self, application_name: str = "xronos", endpoint: str = "localhost:4317"
    ) -> None:
        """Enable collecting and sending telemetry data from the application.

        See :ref:`telemetry` and  :ref:`dashboard` for more information on
        producing, collecting and visualizing telemetry data.

        Args:
            application_name: The name of the application as it should appear
                in the telemetry metadata.
            endpoint: The network endpoint to send telemetry data to. This is
                typically port 4137 on the host running the :ref:`dashboard`.
        """
        return self.__sdk_env.enable_telemetry(application_name, endpoint)


class Reactor(Element):
    """An abstract reactor that can be subclassed to define new reactors."""

    def __pre_init(
        self, name: str, context: sdk.EnvironmentContext | sdk.ReactorContext
    ) -> None:
        self.__name = name
        self.__container_context = context

    def __init__(self) -> None:
        if not hasattr(self, "_Reactor__name") or not hasattr(
            self, "_Reactor__container_context"
        ):
            raise TypeError(
                "Reactors may not be instantiated directly. "
                "Use create_reactor() instead."
            )

        super().__init__(
            sdk.Reactor(self.__name, self.__container_context, lambda: self._assemble())
        )
        self.__initialized = True

        #: Dictionary of contained reactor elements
        self.__elements: dict[str, Element] = {}
        # Initialize all contained elements
        for desc in type(self).__element_descriptors:
            assert desc._name not in self.__elements
            self.__elements[desc._name] = desc._create_instance(self)

        #: List of contained reactors.
        #:
        #: Used for keeping references to all reactors created via
        #: `create_reactor`. This ensures that the garbage collector does not
        #: delete reactors, even if the user does not assign them to a variable or
        #: the variable goes out of scope.
        self.__reactors = list["Reactor"]()

    @property
    def __sdk_instance(self) -> sdk.Reactor:
        return self._get_sdk_instance(sdk.Reactor)

    @classmethod
    def _create_instance(
        cls,
        name: str,
        context: sdk.EnvironmentContext | sdk.ReactorContext,
        *args: Any,
        **kwargs: Any,
    ) -> Self:
        reactor = cls.__new__(cls, *args, **kwargs)
        reactor.__pre_init(name, context)
        reactor.__init__(*args, **kwargs)
        if not hasattr(reactor, "_Reactor__initialized"):
            raise TypeError(
                "super().__init__() must be called when overriding __init__ of Reactor."
            )
        return reactor

    @staticmethod
    def _checked_cast_to_subclass(obj: Callable[..., R]) -> Type[R]:
        """Enforce that the given callable is also a subclass of `Reactor`."""
        # `obj` might be a generic type. We try to get the origin type. If this
        # succeeds its a generic type and we continue with origin, otherwise we
        # continue with obj.
        origin = get_origin(obj)
        cls = origin if origin else obj

        if isinstance(cls, type) and issubclass(cls, Reactor):
            return cast(Type[R], obj)

        raise TypeError("The provided class is not a subclass of xronos.Reactor.")

    @classmethod
    def __init_subclass__(cls) -> None:
        """Initialize any subclass of `Reactor`."""
        super().__init_subclass__()
        cls.__init_attributes()

    @classmethod
    def __init_attributes(cls):
        """Initialize required class attributes.

        This is used to ensure that each subclass keeps track of its
        reactions individually.
        """
        if "_Reactor__element_descriptors" not in cls.__dict__:
            #: List of element descriptors
            cls.__element_descriptors = cls.__get_base_element_descriptors()

    @classmethod
    def __get_base_element_descriptors(cls) -> list["ElementDescriptor[Any]"]:
        """Retrieve the element descriptors of all base classes."""
        descriptors = list["ElementDescriptor[Any]"]()
        base = cls.__get_base_reactor()
        if base:
            for desc in base.__element_descriptors:
                descriptors.append(desc)
        return descriptors

    @classmethod
    def _register_element_descriptor(cls, descriptor: "ElementDescriptor[Any]"):
        """Register an element descriptor."""
        # need to initialize the class attributes as this is called from
        # __set_name__ which is called before __init_submodules.
        cls.__init_attributes()
        assert descriptor not in cls.__element_descriptors
        cls.__element_descriptors.append(descriptor)

    def _get_element(self, name: str) -> Element:
        if not hasattr(self, "_Reactor__initialized"):
            raise TypeError(
                "When overriding __init__ of Reactor, super().__init__() "
                "must be called before any elements are accessed."
            )

        return self.__elements[name]

    @classmethod
    def __get_base_reactor(cls) -> Type["Reactor"] | None:
        """Check bases for other reactor classes.

        Returns:
            The base reactor class of the reactor class currently under
            construction, or None if none is found
        Throws:
            NotImplementedError if the reactor class currently under
            construction inherits from multiple reactor classes.
        """
        base_reactor_cls = None
        for base in cls.__bases__:
            if issubclass(base, Reactor) and base is not Reactor:
                if base_reactor_cls is not None:
                    raise NotImplementedError(
                        "Multiple inheritance for reactor classes is currently not "
                        + "supported!"
                    )
                base_reactor_cls = base
        return base_reactor_cls

    @property
    def startup(self) -> Startup:
        """Startup event source.

        Triggers once when the program execution starts.
        """
        return Startup(self.__sdk_instance.startup)

    @property
    def shutdown(self) -> Shutdown:
        """Shutdown event source.

        Triggers once right before the program execution ends.
        """
        return Shutdown(self.__sdk_instance.shutdown)

    def get_time(self) -> datetime.datetime:
        """Get the current timestamp.

        .. note:: This does not read wall-clock time. The Xronos runtime uses
                  an internal clock to control how a program advances.

        Returns:
            The current timestamp as provided by the internal clock.
        """
        return self.__sdk_instance.get_time()

    def get_lag(self) -> datetime.timedelta:
        """Get the current lag.

        Since in the Xronos SDK time does not advance while reactions execute,
        the internal clock may advance slower than a wall clock would. The lag
        denotes the difference between the wall clock and the internal clock.
        It is a measure of how far the execution of reactions lags behind
        events in the physical world.

        Returns:
            The current lag.
        """
        return self.__sdk_instance.get_lag()

    def get_time_since_startup(self) -> datetime.timedelta:
        """Get the time that passed since the :attr:`startup` event.

        Returns:
            The difference between the current time point given by
            :func:`get_time` and the time at which the program started.
        """
        return self.__sdk_instance.get_time_since_startup()

    def _assemble(self) -> None:
        for _, elem in self.__elements.items():
            if isinstance(elem, Reaction):
                elem._assemble()

    def _context(self, source_location: sdk.SourceLocation) -> sdk.ReactorContext:
        return self.__sdk_instance.context(source_location)

    def _add_reaction(
        self,
        name: str,
        declaration: "Callable[[sdk.ReactionContext], Callable[[], None]]",
        source_location: sdk.SourceLocation,
    ) -> "Reaction":
        sdk_reaction = self.__sdk_instance.add_reaction(name, source_location)
        return Reaction(sdk_reaction, lambda: declaration(sdk_reaction.context()))

    @overload
    def connect(
        self,
        from_: OutputPort[T],
        to: InputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None: ...
    @overload
    def connect(
        self,
        from_: OutputPort[T],
        to: OutputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None: ...
    @overload
    def connect(
        self,
        from_: InputPort[T],
        to: InputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None: ...

    def connect(
        self,
        from_: InputPort[T] | OutputPort[T],
        to: InputPort[T] | OutputPort[T],
        delay: datetime.timedelta | None = None,
    ) -> None:
        """Connect two ports.

        Creates a new connection from the port given in ``from_`` to the port
        given in ``to``.

        If ``delay`` is ``None`` messages are delivered without a delay.
        This means that the timestamp at which the message is received is the
        same as the timestamp at which it was sent.

        If ``delay`` is set messages are delivered with a delay. This means
        that the timestamp at which the message is received is the timestamp at
        which it was sent plus delay.

        Also see `Environment.connect`.

        Args:
            from_(InputPort[T] | OutputPort[T]): The port to draw the
                connection from.
            to(InputPort[T] | OutputPort[T]): The port to draw the connection
                to.
            delay(~datetime.timedelta | None): The delay to apply to messages.
                (optional)

        Raises:
            ValidationError: If an invalid connections is created.
        """
        from_port = from_._get_sdk_instance(sdk.Element)
        to_port = to._get_sdk_instance(sdk.Element)
        assert isinstance(from_port, sdk.InputPort | sdk.OutputPort)
        assert isinstance(to_port, sdk.InputPort | sdk.OutputPort)
        if delay is None:
            self.__sdk_instance.connect(from_port, to_port)
        else:
            self.__sdk_instance.connect_delayed(from_port, to_port, delay)

    def create_reactor(
        self,
        name: str,
        reactor_class: Callable[P, R],
        *args: P.args,
        **kwargs: P.kwargs,
    ) -> R:
        """Create a nested reactor.

        A factory method for instantiating and registering a new nested reactor.
        In contrast to :func:`Environment.create_reactor`, the newly created reactor
        is contained by ``self``.

        Args:
            name: Name of the reactor to be instantiated.
            reactor_class(type): The reactor class to be instantiated. Must be
               a subclass of :class:`~xronos.Reactor`.
            args(~typing.Any): Arguments to be passed to the ``__init__``
                method of ``reactor_class``.
            kwargs(~typing.Any): Keyword arguments to be passed to the
                ``__init__`` method of ``reactor_class``.

        Returns:
            ``reactor_class``:
                 An instance of the class provided in the ``reactor_class``
                 argument.
        """
        # We annotate `reactor_class` as a callable that returns an R to infer
        # the parameters of the init method. `reactor_class` is expected to be
        # a concrete reactor class. The Callable is needed to correctly infer
        # the types of all args and kwargs and support type checking any calls
        # to this method.
        checked_class = Reactor._checked_cast_to_subclass(reactor_class)
        context = self._context(get_source_location())
        reactor = checked_class._create_instance(name, context, *args, **kwargs)

        # keep a reference to the created reactor
        self.__reactors.append(reactor)
        return reactor


class AbsentError(Exception):
    """Indicates an attempt to read a value that is absent.

    See :func:`Trigger.get`.
    """

    pass


class Trigger(Generic[T]):
    """Provides read access to an event source that a reaction is triggered by.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_trigger` instead.
    """

    def __init__(self, sdk_trigger: sdk.Trigger | sdk.VoidTrigger) -> None:
        self.__sdk_trigger = sdk_trigger

    def is_present(self) -> bool:
        """Check if an event is present at the current timestamp."""
        return self.__sdk_trigger.is_present()

    def get(self) -> T:
        """Get the value of a currently present event.

        Raises:
            AbsentError: If called and :func:`is_present` returns ``False``.
        """
        if not self.is_present():
            raise AbsentError(
                "Tried to read a value from an element, but there is no present event."
            )

        if isinstance(self.__sdk_trigger, sdk.Trigger):
            return cast(T, self.__sdk_trigger.get())
        else:
            return cast(T, None)


class PortEffect(Generic[T]):
    """Allows a reaction to write data to a given :class:`InputPort` or
    :class:`OutputPort`.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.

    Type Args:
        T: The value type associated with the port.
    """  # noqa: D205

    def __init__(self, sdk_effect: sdk.PortEffect) -> None:
        self.__sdk_effect = sdk_effect

    def set(self, value: T) -> None:
        """Write a value to the port sending a message to connected ports.

        May be called multiple times, but at most one value is sent to
        connected ports. When called repeatedly at a given timestamp, the
        previous value is overwritten.

        Args:
            value: The value to be written to the referenced port.
        """
        self.__sdk_effect.set(value)

    def is_present(self) -> bool:
        """Check if an event is present at the current Timestamp."""
        return self.__sdk_effect.is_present()

    def get(self) -> T:
        """Get a previously set value.

        Returns:
            The current value if an event is present.

        Raises:
            AbsentError: If called and :func:`is_present` returns ``False``.
        """
        if not self.is_present():
            raise AbsentError(
                "Tried to read a value from an element, but there is no present event."
            )

        return cast(T, self.__sdk_effect.get())


class ProgrammableTimerEffect(Generic[T]):
    """Allows a reaction to schedule future events using a :class:`ProgrammableTimer`.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.

    Type Args:
        T: The value type associated with the programmable timer.
    """

    def __init__(self, sdk_effect: sdk.ProgrammableTimerEffect) -> None:
        self.__sdk_effect = sdk_effect

    def schedule(
        self, value: T, delay: datetime.timedelta = datetime.timedelta(0)
    ) -> None:
        """Schedule a future event.

        Args:
            value: The value to be associated with the event.
            delay: The time to wait until the new event is processed.
        """
        self.__sdk_effect.schedule(value, delay)


class MetricEffect:
    """Allows a reaction to record telemetry data using a given :class:`Metric`.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, sdk_metric: sdk.MetricEffect) -> None:
        self.__sdk_metric = sdk_metric

    def record(self, value: int | float) -> None:
        """Record a value at the current timestamp.

        Args:
            value: The value to be recorded.
        """
        self.__sdk_metric.record(value)


class ShutdownEffect:
    """Allows a reaction to terminate the program.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, sdk_shutdown: sdk.ShutdownEffect) -> None:
        self.__sdk_shutdown = sdk_shutdown

    def trigger_shutdown(self) -> None:
        """Terminate the currently running reactor program.

        Terminates a running program at the next convenience. After completing all
        currently active reactions, this triggers the Shutdown event sources. Once
        all reactions triggered by Shutdown are processed, the program terminates.
        """
        self.__sdk_shutdown.trigger_shutdown()


class ReactionInterface:
    """Helper class for defining the interfaces of a reaction.

    This class is not intended to be instantiated directly. An instance of this
    class is passed automatically to any method decorated with
    {attr}`~xronos.reaction`.
    """

    def __init__(self, context: sdk.ReactionContext) -> None:
        self.__context = context

    def add_trigger(self, event_source: EventSource[T]) -> Trigger[T]:
        """Declare a reaction trigger.

        When the triggering event source emits an event, the reaction that
        declares the trigger will be invoked automatically.

        Args:
            event_source: The event source to declare as the reaction trigger.

        Returns:
            A new trigger object that can be used by the reaction handler to
            check presence and read values.
        """
        sdk_element = event_source._get_sdk_instance(sdk.Element)
        if isinstance(sdk_element, sdk.PeriodicTimer | sdk.Startup | sdk.Shutdown):
            sdk_trigger = sdk.VoidTrigger(sdk_element, self.__context)
        else:
            assert isinstance(
                sdk_element,
                sdk.InputPort
                | sdk.OutputPort
                | sdk.PhysicalEvent
                | sdk.ProgrammableTimer,
            )
            sdk_trigger = sdk.Trigger(sdk_element, self.__context)

        return Trigger(sdk_trigger)

    @overload
    def add_effect(self, target: InputPort[T] | OutputPort[T]) -> PortEffect[T]: ...

    @overload
    def add_effect(
        self, target: ProgrammableTimer[T]
    ) -> ProgrammableTimerEffect[T]: ...

    @overload
    def add_effect(self, target: Metric) -> MetricEffect: ...

    @overload
    def add_effect(self, target: Shutdown) -> ShutdownEffect: ...

    def add_effect(
        self,
        target: InputPort[T] | OutputPort[T] | ProgrammableTimer[T] | Metric | Shutdown,
    ) -> PortEffect[T] | ProgrammableTimerEffect[T] | MetricEffect | ShutdownEffect:
        """Declare a reaction effect.

        An effect provides read and write access to the referenced element, but
        does not trigger the reaction.

        Args:
            target: The reactor element to declare as the reaction effect.

        Returns:
            A new effect object that can be used by the reaction handler to
            write to ports or schedule events.
        """
        if isinstance(target, InputPort | OutputPort):
            sdk_port = target._get_sdk_instance(sdk.Element)
            assert isinstance(sdk_port, sdk.InputPort | sdk.OutputPort)
            return PortEffect(sdk.PortEffect(sdk_port, self.__context))
        elif isinstance(target, ProgrammableTimer):
            sdk_timer = target._get_sdk_instance(sdk.ProgrammableTimer)
            return ProgrammableTimerEffect(
                sdk.ProgrammableTimerEffect(sdk_timer, self.__context)
            )
        elif isinstance(target, Metric):
            sdk_metric = target._get_sdk_instance(sdk.Metric)
            return MetricEffect(sdk.MetricEffect(sdk_metric, self.__context))
        elif isinstance(target, Shutdown):  # type: ignore [reportUnnecessaryIsInstance]
            sdk_shutdown = target._get_sdk_instance(sdk.Shutdown)
            return ShutdownEffect(sdk.ShutdownEffect(sdk_shutdown, self.__context))
        else:
            raise ValueError(f"{type(target)} is not a valid target for an effect.")


class Reaction(Element):
    def __init__(
        self,
        sdk_instance: sdk.Reaction,
        declaration: Callable[[], Callable[[], None]],
    ) -> None:
        super().__init__(sdk_instance)
        self.__declaration = declaration

    def _assemble(self):
        sdk_reaction = self._get_sdk_instance(sdk.Reaction)
        handler = self.__declaration()
        sdk_reaction.set_handler(handler)

    def __call__(self) -> None:
        raise RuntimeError("Reactions may not be called directly")


class ReactionDescriptor(ElementDescriptor[Reaction], Generic[R]):
    """A descriptor for reactions.

    This is an implementation of the Python descriptor protocol. It is used
    to manage the declaration and instantiation of reactions. The complete
    protocol is split between this class and the `Reactor` base class.

    Reaction descriptors are instantiated when the `@reaction` decorator is
    used (see :func:`reaction`). The annotated function is expected to
    accept a reactor instance and a reaction interface as arguments and to
    return a reaction handler. The function is expected to use the given
    reactor interface to declare the reaction's dependencies.

    Args:
        declaration: A callable that accepts a reactor and reaction
            interface as arguments and returns a reaction handler. The
            callable responsible for declaring reaction dependencies using
            the provide reaction interface.
        source_location: source location to be associated with the reaction
    """

    def __init__(
        self,
        declaration: Callable[[R, ReactionInterface], Callable[[], None]],
        source_location: sdk.SourceLocation,
    ):
        def initialize(name: str, reactor: Reactor) -> Reaction:
            return reactor._add_reaction(
                name,
                lambda context: declaration(
                    cast(R, reactor), ReactionInterface(context)
                ),
                source_location,
            )

        super().__init__(initialize, None)

    def __call__(self) -> None:
        raise RuntimeError("Reactions may not be called directly")


def reaction(
    declaration: Callable[[R, ReactionInterface], Callable[[], None]],
) -> ReactionDescriptor[R]:
    """Decorator that is used to declare reactions.

    Args:
        declaration: The decorated method. Must accept an
            :class:`~xronos.ReactionInterface` as its first argument and return
            a reaction handler. Failing to return a handler will result in an
            exception when the reactor containing the reaction is initialized.
    """
    return ReactionDescriptor[R](declaration, get_source_location())

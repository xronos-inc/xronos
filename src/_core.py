# SPDX-FileCopyrightText: © 2024 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# Ignore access to protected/private members. We use access to protected
# members of other classes to implement accessor objects.
# pyright: reportPrivateUsage = false
# Ignore missing source for xronos._runtime
# pyright: reportMissingModuleSource = false

import datetime
import inspect
from typing import (
    Any,
    Callable,
    Generic,
    Optional,
    ParamSpec,
    Type,
    TypeAlias,
    TypeVar,
    cast,
    overload,
)

# Self is only available for Python>=3.11
try:
    from typing import Self
except ImportError:
    from typing_extensions import Self

import xronos._runtime as runtime
from xronos._runtime import (
    ValidationError,  # pyright: ignore[reportUnusedImport]
)

ValidationError.__doc__ = (
    "An exception that indicates problems in the reactor topology as defined by"
    " the program."
)

T = TypeVar("T")


def _get_calling_stack_frame() -> inspect.FrameInfo:
    stack = inspect.stack()
    # The frame that we are looking for is 2 frames up the stack. 1 up gets to
    # the caller of this function and 2 up gets to their caller.
    frame = stack[2]
    # In case of generics there might be another level of indirection and we
    # have to go one more frame up.
    if frame.filename.endswith("typing.py"):
        frame = stack[3]
    return frame


Elem = TypeVar("Elem", bound=runtime.ReactorElement)


class ElementDescriptor(Generic[Elem]):
    """A descriptor for a reactor element.

    Args:
        initializer: a callable that can be used to create the element
        frame_info: stack frame to be associated with the element's source location
    """

    def __init__(
        self,
        initializer: Callable[[str, "Reactor"], Elem],
        frame_info: inspect.FrameInfo,
    ) -> None:
        self.__name: str | None = None
        self.__initializer = initializer
        self.__frame_info = frame_info

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

    def _get_instance(self, container: "Reactor"):
        """Return an instance of the described element.

        Args:
            container: the owning `Reactor`
        """
        instance = self.__initializer(self._name, container)
        container.environment._add_frame_info(self.__frame_info, instance)
        return instance


class Startup(runtime.Startup):
    """An event that triggers when the program starts executing."""

    @property
    def name(self) -> str:
        """Name of the startup event (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the startup event (read-only)."""
        return self._fqn


class StartupDeclaration(ElementDescriptor[Startup]):
    def __init__(self) -> None:
        super().__init__(Startup, frame_info=_get_calling_stack_frame())


class Shutdown(runtime.Shutdown):
    """An event that triggers before the program shuts down."""

    @property
    def name(self) -> str:
        """Name of the shutdown event (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the shutdown event (read-only)."""
        return self._fqn


class ShutdownDeclaration(ElementDescriptor[Shutdown]):
    def __init__(self) -> None:
        super().__init__(Shutdown, frame_info=_get_calling_stack_frame())


class Timer(runtime.Timer):
    """A trigger that emits events in regular intervals."""

    @property
    def name(self) -> str:
        """Name of the timer (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the timer (read-only)."""
        return self._fqn

    @property
    def period(self) -> datetime.timedelta:
        """The delay in between two events emitted by the timer (read-write).

        If :attr:`period` is 0 (the default), then the timer will trigger only
        once with a delay of :attr:`offset` after :attr:`~Reactor.startup`
        triggers. If :attr:`offset` is also set to 0, the timer triggers
        simultaneously to :attr:`~Reactor.startup`.
        """
        return self._period

    @period.setter
    def period(self, value: datetime.timedelta) -> None:
        self._period = value

    @property
    def offset(self) -> datetime.timedelta:
        """The delay between :attr:`~Reactor.startup` and the first event emitted by the timer (read-write)."""  # noqa: E501
        return self._offset

    @offset.setter
    def offset(self, value: datetime.timedelta) -> None:
        self._offset = value


class TimerDeclaration(ElementDescriptor[Timer]):
    """A declaration for a :class:`Timer`.

    Args:
        period: The delay in between two events emitted by the timer (optional).
        offset: The delay between the :attr:`~Reactor.startup` event
            and the first event emitted by the timer (optional).
    """

    def __init__(
        self,
        period: datetime.timedelta = datetime.timedelta(0),
        offset: datetime.timedelta = datetime.timedelta(0),
    ) -> None:
        super().__init__(
            lambda name, container: Timer(name, container, period, offset),
            frame_info=_get_calling_stack_frame(),
        )


class InputPort(Generic[T], runtime.Input):
    """A port that receives values from other reactors.

    The input port does not provide direct access to received values. A
    :attr:`reaction` may declare an input port as a :class:`Trigger` or
    :class:`Source` to read its value.

    Type Parameters:
        T(optional): The type of values carried by the port.
    """

    @property
    def name(self) -> str:
        """Name of the input port (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the input port (read-only)."""
        return self._fqn

    def _set(self, value: T) -> None:
        super()._set(value)

    def _get(self) -> T:
        return cast(T, super()._get())


class InputPortDeclaration(ElementDescriptor[InputPort[T]]):
    """A declaration for an :class:`InputPort[T]<InputPort>`.

    Type Parameters:
        T(optional): The type of values carried by the port.
    """

    def __init__(self) -> None:
        super().__init__(InputPort, frame_info=_get_calling_stack_frame())


class OutputPort(Generic[T], runtime.Output):
    """A port that sends values to other reactors.

    The output port does not provide direct access for writing values. A
    :attr:`reaction` may declare a :class:`PortEffect` to send a value.

    Type Parameters:
        T(optional): The type of values carried by the port.
    """

    @property
    def name(self) -> str:
        """Name of the output port (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the output port (read-only)."""
        return self._fqn

    def _set(self, value: T) -> None:
        super()._set(value)

    def _get(self) -> T:
        return cast(T, super()._get())


class OutputPortDeclaration(ElementDescriptor[OutputPort[T]]):
    """A declaration for an :class:`OutputPort[T]<OutputPort>`.

    Type Parameters:
        T(optional): The type of values carried by the port.
    """

    def __init__(self) -> None:
        super().__init__(OutputPort, frame_info=_get_calling_stack_frame())


class InternalEvent(Generic[T], runtime.InternalEvent):
    """An element for scheduling new events.

    Internal events may be used by reactions to schedule new events in the
    future. Events may not be scheduled or read directly. Instead,
    :attr:`reaction`'s may declare an :class:`InternalEventEffect` to schedule
    new events, or a :class:`Trigger` or :class:`Source` to access the value
    associated with an active event.

    Type Parameters:
        T(optional): The type of values carried by the internal event.
    """

    @property
    def name(self) -> str:
        """Name of the internal event (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the internal event (read-only)."""
        return self._fqn

    def _schedule(self, value: T, delay: datetime.timedelta) -> None:
        super()._schedule(value, delay)

    def _get(self) -> T:
        return cast(T, super()._get())


class InternalEventDeclaration(ElementDescriptor[InternalEvent[T]]):
    """A declaration for a :class:`InternalEvent[T]<InternalEvent>`.

    Type Parameters:
        T(optional): The type of values carried by the internal event.
    """

    def __init__(self) -> None:
        super().__init__(InternalEvent, frame_info=_get_calling_stack_frame())


class PhysicalEvent(Generic[T], runtime.PhysicalEvent):
    """An element for scheduling new events from external contexts.

    Physical events may be used to schedule new events from an external context
    outside of the scope of the reactor program. These could be external event
    handlers that respond to sensor inputs. Physical events do not provide
    direct access to their values. A :attr:`reaction` may declare a
    :class:`Trigger` or :class:`Source` to access the value associated with an
    active event.

    Type Parameters:
        T(optional): The type of values carried by the event.
    """

    @property
    def name(self) -> str:
        """Name of the physical event (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the physical event (read-only)."""
        return self._fqn

    def schedule(self, value: T) -> None:
        """Create a new event.

        This will automatically assign a timestamp to the newly created event.

        Args:
            value: A value to be associated with the newly created event.
        """
        self._schedule(value, datetime.timedelta(0))

    def _get(self) -> T:
        return cast(T, super()._get())


class PhysicalEventDeclaration(ElementDescriptor[PhysicalEvent[T]]):
    """A declaration for a :class:`PhysicalEvent[T]<PhysicalEvent>`.

    Type Parameters:
        T(optional): The type of values carried by the physical event.
    """

    def __init__(self) -> None:
        super().__init__(PhysicalEvent, frame_info=_get_calling_stack_frame())


Port: TypeAlias = InputPort[T] | OutputPort[T]

Container: TypeAlias = runtime.Environment | runtime.Reactor


R = TypeVar("R", bound="Reactor")
P = ParamSpec("P")


class Environment(runtime.Environment):
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
        self,
        fast: bool = False,
        timeout: Optional[datetime.timedelta] = None,
    ) -> None:
        if timeout:
            super().__init__(workers=1, fast=fast, timeout=timeout)
        else:
            super().__init__(workers=1, fast=fast)

        #: List of all top-level reactors.
        #:
        #: Used for keeping references to all reactors created via
        #: `create_reactor`. This ensures that the garbage collector does not
        #: delete reactors, even if the user does not assign them to a variable or
        #: the variable goes out of scope.
        self.__reactors = list["Reactor"]()

        #: List of source infos that map reactor elements to their location of
        #: creation in the source code.
        self.__source_locations = list[runtime.SourceInfo]()

    def execute(self) -> None:
        """Execute the reactor program.

        Initiates the execution of a reactor program. This triggers
        :attr:`~xronos.Reactor.startup` and instructs the runtime to start
        processing reactions.

        Returns when the reactor program terminates.
        """
        for reactor in self._top_level_reactors:
            Environment._recursive_init(reactor)
        super()._execute(self.__source_locations)

    @staticmethod
    def _recursive_init(reactor: runtime.Reactor) -> None:
        if isinstance(reactor, Reactor):
            reactor._initialize()
        for child in reactor._reactor_instances:
            Environment._recursive_init(child)

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
        self, from_: Port[T], to: Port[T], delay: datetime.timedelta | None = None
    ) -> None:
        """Connect two ports.

        Creates a new connection from the port given in ``from_`` to the port
        given in ``to``.

        Args:
            from_(InputPort[T] | OutputPort[T]): The port to draw the
                connection from.
            to(InputPort[T] | OutputPort[T]): The port to draw the connection
                to.
            delay(~datetime.timedelta | None): When set, the connection waits
                for `delay` before delivering the messages to `to`. (optional)
        """
        if delay is None:
            super()._connect(from_, to)
        else:
            super()._connect_delayed(from_, to, delay)

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
        reactor = cast(Type[R], reactor_class)._create_instance(
            name, self, _get_calling_stack_frame(), *args, **kwargs
        )
        # keep a reference to the created reactor
        self.__reactors.append(reactor)
        return reactor

    @staticmethod
    def __frameinfo_to_sourceinfo(
        frame: inspect.FrameInfo, element: runtime.ReactorElement
    ):
        try:
            pos = frame.positions
        except AttributeError:
            # Fallback for Python 3.10
            pos = None
        lineno = pos.lineno if pos and pos.lineno else frame.lineno
        end_lineno = pos.end_lineno if pos and pos.end_lineno else lineno
        col_offset = pos.col_offset if pos and pos.col_offset else 0
        end_col_offset = pos.end_col_offset if pos and pos.end_col_offset else 80
        return runtime.SourceInfo(
            type(element).__name__,
            frame.function,
            frame.filename,
            element._fqn.split("."),
            element._uid,
            lineno,
            end_lineno,
            col_offset,
            end_col_offset,
        )

    def _add_frame_info(
        self, frame: inspect.FrameInfo, element: runtime.ReactorElement
    ):
        source_info = Environment.__frameinfo_to_sourceinfo(frame, element)
        self.__source_locations.append(source_info)

    @staticmethod
    def enable_tracing(
        endpoint: str = "localhost:4317",
        application_name: str = "xronos",
    ) -> None:
        """Enable the collection of trace data during program execution.

        See :ref:`dashboard` for information on how to visualize the trace data.

        Args:
            endpoint: The network endpoint to send trace data to. This is
                typically port 4137 on the host running the :ref:`dashboard`.
            application_name: The name of the application as it should appear
                in the trace data.
        """
        runtime.Environment.enable_tracing(
            endpoint=endpoint, application_name=application_name
        )

    def request_shutdown(self) -> None:
        """Request the termination of a currently running reactor program.

        Terminates a program started with :func:`execute` at the next
        convenience. This triggers :attr:`~xronos.Reactor.shutdown` after
        completing all currently active reactions, and stops program execution after
        processing all reactions triggered by :attr:`~xronos.Reactor.shutdown`.
        """
        return super().request_shutdown()


class Reactor(runtime.Reactor):
    """An abstract reactor that can be subclassed to define new reactors."""

    #: Startup: Startup event.
    #:
    #: Triggers once when the program execution starts.
    startup = StartupDeclaration()

    #: Shutdown: Shutdown event.
    #:
    #: Triggers once right before the program execution ends.
    shutdown = ShutdownDeclaration()

    def __pre_init(self, name: str, container: Container) -> None:
        self.__name = name
        self.__container = container

    def __init__(self) -> None:
        super().__init__(self.__name, self.__container)
        self.__initialized = True

        self._reactions = dict[str, Reaction]()

        #: Dictionary of contained reactor elements
        self.__elements: dict[str, runtime.ReactorElement] = {}
        # Initialize all contained elements
        for desc in type(self).__element_descriptors:
            assert desc._name not in self.__elements
            self.__elements[desc._name] = desc._get_instance(self)

        #: List of contained reactors.
        #:
        #: Used for keeping references to all reactors created via
        #: `create_reactor`. This ensures that the garbage collector does not
        #: delete reactors, even if the user does not assign them to a variable or
        #: the variable goes out of scope.
        self.__reactors = list["Reactor"]()

    @property
    def name(self) -> str:
        """Name of the reactor (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the reactor (read-only)."""
        return self._fqn

    @classmethod
    def _create_instance(
        cls,
        name: str,
        container: Container,
        frame: inspect.FrameInfo,
        *args: Any,
        **kwargs: Any,
    ) -> Self:
        reactor = cls.__new__(cls, *args, **kwargs)
        reactor.__pre_init(name, container)
        reactor.__init__(*args, **kwargs)
        if not hasattr(reactor, "_Reactor__initialized"):
            raise TypeError(
                "super().__init__() must be called when overriding __init__ of Reactor."
            )
        reactor.environment._add_frame_info(frame, reactor)
        return reactor

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
        if "_Reactor__reaction_priority_counter" not in cls.__dict__:
            cls.__reaction_priority_counter = cls.__get_base_priority()
        if "_Reactor__reaction_descriptors" not in cls.__dict__:
            cls.__reaction_descriptors = cls.__get_base_reaction_descriptors()
        if "_Reactor__element_descriptors" not in cls.__dict__:
            #: List of element descriptors
            cls.__element_descriptors = cls.__get_base_element_descriptors()

    @classmethod
    def __get_base_element_descriptors(cls) -> set["ElementDescriptor[Any]"]:
        """Retrieve the element descriptors of all base classes."""
        descriptors = set["ElementDescriptor[Any]"]()
        base = cls.__get_base_reactor()
        if base:
            descriptors.update(base.__element_descriptors)
        return descriptors

    @classmethod
    def _register_element_descriptor(cls, descriptor: "ElementDescriptor[Any]"):
        """Register an element descriptor."""
        # need to initialize the class attributes as this is called from
        # __set_name__ which is called before __init_submodules.
        cls.__init_attributes()
        assert descriptor not in cls.__element_descriptors
        cls.__element_descriptors.add(descriptor)

    def _get_element(self, name: str) -> runtime.ReactorElement:
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
            if issubclass(base, runtime.Reactor) and base is not runtime.Reactor:
                if base_reactor_cls is not None:
                    raise NotImplementedError(
                        "Multiple inheritance for reactor classes is currently not "
                        + "supported!"
                    )
                base_reactor_cls = cast(Type["Reactor"], base)
        return base_reactor_cls

    @classmethod
    def __get_base_priority(cls) -> int:
        """Get the priority counter of the base reactor class.

        Returns:
            The priority counter value of the base reactor class, or 0 if there
            is no base.
        """
        base_reactor = cls.__get_base_reactor()
        return base_reactor.__reaction_priority_counter if base_reactor else 0

    @classmethod
    def __get_base_reaction_descriptors(cls) -> list["ReactionDescriptor[Reactor]"]:
        """Get the list of reaction descriptors of the base reactor class.

        Returns:
            The list of reaction descriptors of the base reactor class, or an
            empty list if there is no base.
        """
        base_reactor = cls.__get_base_reactor()
        return base_reactor.__reaction_descriptors.copy() if base_reactor else []

    @classmethod
    def _register_reaction_descriptor(
        cls, descriptor: "ReactionDescriptor[Reactor]"
    ) -> int:
        """Register a reaction descriptor.

        This is intended to be called during `__set_name__` of
        :class:`ReactionDescriptor`. It adds the descriptor to
        `_reaction_descriptors`, increments `_reaction_priority_counter` and
        returns the resulting priority.

        Args:
            descriptor: The reaction descriptor to register

        Returns:
            The priority of the newly registered reaction.
        """
        cls.__init_attributes()
        cls.__reaction_priority_counter += 1
        cls.__reaction_descriptors.append(descriptor)
        return cls.__reaction_priority_counter

    @property
    def environment(self) -> Environment:
        """The environment that the reactor is associated with (read-only)."""
        env = super()._environment
        assert isinstance(env, Environment)
        return env

    def get_time(self) -> datetime.datetime:
        """Get the current point in time.

        .. note:: This does not read wall-clock time. The ``xronos`` runtime
                  uses an internal clock to control how a program advances.
                  :func:`get_time` reads the current time as measured by the
                  internal clock.
        """
        return self._get_logical_time()

    def get_lag(self) -> datetime.timedelta:
        """Get the current lag.

        Since time in the ``xronos`` runtime does not advance while reactions
        execute, the internal clock may advance slower than a wall clock would.
        The lag denotes the difference between the wall clock and the internal
        clock. It is a measure of how far the execution of reactions lags behind
        events in the physical world.
        """
        return self._get_physical_time() - self._get_logical_time()

    def get_time_since_startup(self) -> datetime.timedelta:
        """Get the time that passed since the :attr:`startup` event.

        Returns:
            The difference between the current time point given by
            :func:`get_time`, and the time at which the program started.
        """
        return self._get_elapsed_logical_time()

    def _assemble(self) -> None:
        for reaction in self._reactions.values():
            reaction.assemble()

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
        self, from_: Port[T], to: Port[T], delay: datetime.timedelta | None = None
    ) -> None:
        """Connect two ports.

        See :func:`Environment.connect`.
        """
        # Ignore typing here. Alternatively we could use runtime assertions to
        # check the types, but that is unnecessary as `Reactor.connect` uses
        # the same overloads as `Environment.connect`, and the underlying
        # implementation in the C++ runtime already performs such assertions.
        self.environment.connect(from_, to, delay)  # pyright: ignore

    def _initialize(self) -> None:
        """Initialize the reactor.

        Ensures that all reactions are instantiated for this particular reactor
        instance.
        """
        for descriptor in type(self).__reaction_descriptors:
            reaction = descriptor.create_instance(self)
            self._reactions[descriptor.name] = reaction

    def create_reactor(
        self,
        name: str,
        reactor_class: Callable[P, R],
        *args: P.args,
        **kwargs: P.kwargs,
    ) -> R:
        """Create a new nested reactor.

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
        reactor = cast(Type[R], reactor_class)._create_instance(
            name, self, _get_calling_stack_frame(), *args, **kwargs
        )
        # keep a reference to the created reactor
        self.__reactors.append(reactor)
        return reactor


TypedEventSource: TypeAlias = Port[T] | InternalEvent[T] | PhysicalEvent[T]

ValuelessEventSource: TypeAlias = Timer | Startup | Shutdown

GenericEventSource: TypeAlias = TypedEventSource[T] | ValuelessEventSource


class _Dependency(Generic[T]):
    def __init__(self, element: GenericEventSource[T]) -> None:
        self._element = element

    @property
    def is_present(self) -> bool:
        """Indicates if the referenced element has triggered."""
        return self._element._is_present


class Source(Generic[T], _Dependency[T]):
    """An accessor for a reactor element that is a reaction source.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_source` instead.
    """

    def __init__(self, element: GenericEventSource[T]) -> None:
        super().__init__(element)

    @property
    def value(self) -> T:
        """The current value of the referenced reactor element (read-only).

        Raises:
            AttributeError: If the referenced element does not have a value
                (e.g. :class:`~xronos.Timer`, :class:`~xronos.Shutdown`,
                :class:`~xronos.Startup`)
            AttributeError: If accessed and :attr:`is_present` is ``False``.
        """
        if not self.is_present:
            raise AttributeError("Cannot access value as it is not present")
        if isinstance(
            self._element,
            (runtime.Timer, runtime.Startup, runtime.Shutdown),
        ):
            raise AttributeError("The element does not have a value")
        return self._element._get()


class Trigger(Generic[T], Source[T]):
    """An accessor for a reactor element that is a reaction trigger.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_trigger` instead.
    """

    pass


class PortEffect(Generic[T], Source[T]):
    """An accessor for a reactor element that is a reaction effect on a port.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, element: Port[T]) -> None:
        super().__init__(element)
        # overwrite so that type checker knows element is a Port[T]
        self._element = element

    @property
    def value(self) -> T:
        """The current value of the referenced port (read-write).

        May be written to send a value via the port. For each execution of a
        reaction only one the last value written will be sent.

        Raises:
            AttributeError: If read and :attr:`is_present` is ``False``.
        """
        return super().value

    @value.setter
    def value(self, v: T) -> None:
        self._element._set(v)


class InternalEventEffect(Generic[T], Source[T]):
    """An accessor for a reactor element that is a reaction effect on an internal event.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, element: InternalEvent[T]) -> None:
        super().__init__(element)
        # overwrite so that type checker knows element is an InternalEvent[T]
        self._element = element

    def schedule(
        self, value: T, delay: datetime.timedelta = datetime.timedelta(0)
    ) -> None:
        """Schedule a new occurrence of the referenced event.

        Args:
            value: The value to be associated with the event occurrence.
            delay: The time to wait until the new event is processed.
        """
        self._element._schedule(value, delay)


class ReactionInterface:
    """Helper class for defining the interfaces of a reaction.

    This class is not intended to be instantiated directly. An instance of this
    class is passed automatically to any method decorated with
    {attr}`~xronos.reaction`.
    """

    def __init__(self) -> None:
        self._triggers = set[GenericEventSource[Any]]()
        self._sources = set[GenericEventSource[Any]]()
        self._effects = set[Port[Any] | InternalEvent[Any]]()

    @overload
    def add_trigger(self, trigger: TypedEventSource[T]) -> Trigger[T]: ...

    @overload
    def add_trigger(self, trigger: ValuelessEventSource) -> Trigger[None]: ...

    def add_trigger(self, trigger: GenericEventSource[T]) -> Trigger[T]:
        """Declare a reaction trigger.

        When the trigger element is activated by an event, the reaction that
        declares the trigger will be invoked automatically.

        Args:
            trigger: The reactor element to declare as the reaction trigger.

        Returns:
            A new trigger object that can be used by the reaction handler to
            check presence and read values.
        """
        self._triggers.add(trigger)
        return Trigger(trigger)

    @overload
    def add_source(self, source: TypedEventSource[T]) -> Source[T]: ...

    @overload
    def add_source(self, source: Timer | Startup | Shutdown) -> Source[None]: ...

    def add_source(self, source: GenericEventSource[T]) -> Source[T]:
        """Declare a reaction source.

        A source provides read access to the referenced element but does not
        trigger the reaction.

        Args:
            source: The reactor element to declare as the reaction source.

        Returns:
            A new source object that can be used by the reaction handler to
            check presence and read values.
        """
        self._sources.add(source)
        return Source(source)

    @overload
    def add_effect(self, effect: Port[T]) -> PortEffect[T]: ...

    @overload
    def add_effect(self, effect: InternalEvent[T]) -> InternalEventEffect[T]: ...

    def add_effect(
        self, effect: Port[T] | InternalEvent[T]
    ) -> PortEffect[T] | InternalEventEffect[T]:
        """Declare a reaction effect.

        An effect provides read and write access to the referenced element, but
        does not trigger the reaction.

        Args:
            effect: The reactor element to declare as the reaction effect.

        Returns:
            A new effect object that can be used by the reaction handler to
            write to ports or schedule events.
        """
        self._effects.add(effect)
        if isinstance(effect, InputPort | OutputPort):
            return PortEffect(effect)
        else:
            return InternalEventEffect(effect)


class Reaction(runtime.Reaction):
    def __init__(
        self,
        name: str,
        priority: int,
        container: Reactor,
        handler: Callable[[], None],
        interface: ReactionInterface,
    ) -> None:
        super().__init__(name, priority, container, handler)
        self._handler = handler
        self._interface = interface

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        raise RuntimeError("Reactions may not be called directly.")

    @property
    def name(self) -> str:
        """Name of the reaction (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the reaction (read-only)."""
        return self._fqn

    def assemble(self) -> None:
        for trigger in self._interface._triggers:
            if isinstance(trigger, runtime.EventSource):
                self._declare_event_source_trigger(trigger)
            else:
                self._declare_port_trigger(trigger)
        for effect in self._interface._effects:
            if isinstance(effect, runtime.EventSource):
                self._declare_event_source_effect(effect)
            else:
                self._declare_port_effect(effect)
        for source in self._interface._sources:
            if isinstance(source, runtime.EventSource):
                raise NotImplementedError(
                    "At the moment, only ports are supported as reaction sources."
                )
            else:
                self._declare_port_source(source)


class ReactionDescriptor(Generic[R]):
    def __init__(
        self,
        declaration_func: Callable[[R, ReactionInterface], Callable[[], None]],
        frame_info: inspect.FrameInfo,
    ):
        """A descriptor for reactions.

        This is an implementation of the Python descriptor protocol. It is used
        to manage the deceleration and instantiation of reactions. The complete
        protocol is split between this class and the `Reactor` base class.

        Reaction descriptors are instantiated when the `@reaction` decorator is
        used (see :func:`reaction`). The annotated function is expected to
        accept a reactor instance and a reaction interface as arguments and to
        return a reaction handler. The function is expected to use the given
        reactor interface to declare the reaction's dependencies.

        Python's descriptor protocol ensures that `__set_name__` is called once
        during construction of the containing (reactor) class. We use this, to
        record the reaction name and register the descriptor using
        `_register_reaction_descriptor` provided by each reactor class.
        `_register_reaction_descriptor` also returns
        a priority, which the descriptor records for later use.

        During initialization `_initialize` of :class:`Reactor` is called. This
        will call `create_instance` on each recorded reaction descriptor.
        `create_instance` uses the recorded information and returns a new
        `Reaction` instance. The calling `Reactor` instance is responsible for
        storing the reaction instance in its `_reactions` dict. If `__get__` is
        called on the descriptor, it retrieves the concrete instances from the
        `_reactions` dict of the calling reactor.

        Args:
            declaration_func: A callable that accepts a reactor and reaction
                interface as arguments and returns a reaction handler. The
                callable responsible for declaring reaction dependencies using
                the provide reaction interface.
            frame_info: stack frame to be associated with the element's source
                location

        They are intended to use the reaction functionality.
        """
        self._deceleration_func = declaration_func
        self._name: Optional[str] = None
        self._priority: Optional[int] = None
        self._instances: dict[Reactor, Reaction] = {}
        self.__frame_info = frame_info

    @property
    def name(self) -> str:
        """The reaction name."""
        assert self._name
        return self._name

    @property
    def priority(self) -> int:
        """The reaction priority."""
        assert self._priority
        return self._priority

    def __set_name__(self, reactor_cls: Type[R], name: str) -> None:
        """Register the reaction and record its name and priority.

        This is called automatically during the creation of a reactor class.
        For instance, given the following reactor
        ```
        class Foo(xronos.Reactor):
            @xronos.reaction
            def bar(self, interface): ...
        ```
        this method will be called with `Foo` and `"bar"` as arguments during
        the creation of `Foo`.
        """
        assert self._name is None
        assert self._priority is None
        self._name = name
        self._priority = reactor_cls._register_reaction_descriptor(
            cast("ReactionDescriptor[Reactor]", self)
        )

    def __get__(self, reactor: R, _: Any = None) -> Reaction:
        """Return a reaction instance that is owned by the given reactor."""
        return reactor._reactions[self.name]

    def create_instance(self, reactor: R) -> Reaction:
        """Create a new reaction instance that is owned by the given reactor.

        This is a factory method that instantiates a reaction using the
        information recorded in this descriptor. The given reactor is
        registered as the owner of the reaction.
        """
        interface = ReactionInterface()
        handler: Callable[[], None] = self._deceleration_func(reactor, interface)
        reaction = Reaction(
            name=self.name,
            priority=self.priority,
            container=reactor,
            handler=handler,
            interface=interface,
        )
        reactor.environment._add_frame_info(self.__frame_info, reaction)
        return reaction

    def __call__(self) -> None:
        raise RuntimeError("Reactions may not be called directly")


def reaction(
    declaration: Callable[[R, ReactionInterface], Callable[[], None]],
) -> Callable[[], None]:
    """Decorator that is used to declare reactions.

    Args:
        declaration: The decorated method. Must accept an
            :class:`~xronos.ReactionInterface` as its first argument and return
            a reaction handler.
    """
    return ReactionDescriptor[R](declaration, _get_calling_stack_frame())

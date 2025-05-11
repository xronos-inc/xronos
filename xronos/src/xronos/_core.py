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
import platform
import sys
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
    get_origin,
    overload,
)

# Self is only available for Python>=3.11
try:
    from typing import Self
except ImportError:
    from typing_extensions import Self

# deprecated is only available for Python>=3.13
try:
    from warnings import deprecated
except ImportError:
    from typing_extensions import deprecated

import xronos._runtime as runtime
from xronos._runtime import TelemetryBackend, ValidationError

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


AttributeValue: TypeAlias = str | bool | int | float
AttributeMap: TypeAlias = dict[str, AttributeValue]


class AttributeMixin:
    def add_attribute(self, key: str, value: AttributeValue) -> None:
        """Annotate the element with an attribute.

        See :ref:`attributes` for more information.

        Args:
            key: The name of the attribute to add
            value: The value of the attribute to add

        Raises:
            KeyError: If the attribute key was added before.
        """
        assert isinstance(self, runtime.ReactorElement)
        result = self._environment._add_attribute(self, key, value)
        if not result:
            raise KeyError("Overwriting an existing attribute is not permitted.")

    def add_attributes(self, attributes: AttributeMap) -> None:
        """Annotate the element with multiple attributes.

        See :ref:`attributes` for more information.

        Args:
            attributes: A map of attribute names and their values to be added.

        Raises:
            KeyError: If any of the provided attribute keys was added before.
        """
        assert isinstance(self, runtime.ReactorElement)
        result = self._environment._add_attributes(self, attributes)
        if not result:
            raise KeyError("Overwriting an existing attribute is not permitted.")


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
        attributes: AttributeMap | None = None,
    ) -> None:
        self.__name: str | None = None
        self.__initializer = initializer
        self.__frame_info = frame_info
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

    def _get_instance(self, container: "Reactor") -> runtime.ReactorElement:
        """Return an instance of the described element.

        Args:
            container: the owning `Reactor`
        """
        instance = self.__initializer(self._name, container)
        container.environment._add_frame_info(self.__frame_info, instance)
        if self.__attributes:
            assert isinstance(instance, AttributeMixin)
            instance.add_attributes(self.__attributes)
        return instance


class Startup(runtime.Startup, AttributeMixin):
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


class Shutdown(runtime.Shutdown, AttributeMixin):
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


class PeriodicTimer(runtime.Timer, AttributeMixin):
    """A trigger that emits events in regular intervals."""

    @property
    def name(self) -> str:
        """Name of the periodic timer (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the periodic timer (read-only)."""
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
        super().__init__(
            lambda name, container: PeriodicTimer(name, container, period, offset),
            frame_info=_get_calling_stack_frame(),
            attributes=attributes,
        )


class InputPort(Generic[T], runtime.Input, AttributeMixin):
    """A port that receives values from other reactors.

    The input port does not provide direct access to received values. A
    :attr:`reaction` may declare an input port as a :class:`Trigger` or
    :class:`Effect` to read or write its value.

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

    Args:
        attributes(optional): A dict of attributes characterizing the input port.

    Type Parameters:
        T(optional): The type of values carried by the port.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        super().__init__(
            InputPort, frame_info=_get_calling_stack_frame(), attributes=attributes
        )


class OutputPort(Generic[T], runtime.Output, AttributeMixin):
    """A port that sends values to other reactors.

    The output port does not provide direct access to received values. A
    :attr:`reaction` may declare an output port as a :class:`Trigger` or
    :class:`Effect` to read or write its value.

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

    Args:
        attributes(optional): A dict of attributes characterizing the output port.

    Type Parameters:
        T(optional): The type of values carried by the port.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        super().__init__(
            OutputPort, frame_info=_get_calling_stack_frame(), attributes=attributes
        )


class ProgrammableTimer(Generic[T], runtime.ProgrammableTimer, AttributeMixin):
    """An element for scheduling future events.

    Programmable timers may be used by reactions to schedule new events in the
    future. Events are not scheduled or read directly. Instead,
    :attr:`reaction`'s may declare a :class:`ProgrammableTimerEffect` to
    schedule new events, or a :class:`Trigger` to access the
    value associated with an active event.

    Type Parameters:
        T(optional): The type of values carried by the programmable times.
    """

    @property
    def name(self) -> str:
        """Name of the programmable timer (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the programmable timer (read-only)."""
        return self._fqn

    def _schedule(self, value: T, delay: datetime.timedelta) -> None:
        super()._schedule(value, delay)

    def _get(self) -> T:
        return cast(T, super()._get())


class ProgrammableTimerDeclaration(ElementDescriptor[ProgrammableTimer[T]]):
    """A declaration for a :class:`ProgrammableTimer[T]<ProgrammableTimer>`.

    Args:
        attributes(optional): A dict of attributes characterizing the
        programmable timer.

    Type Parameters:
        T(optional): The type of values carried by the programmable timer.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        super().__init__(
            ProgrammableTimer,
            frame_info=_get_calling_stack_frame(),
            attributes=attributes,
        )


class PhysicalEvent(Generic[T], runtime.PhysicalEvent, AttributeMixin):
    """An element for triggering events from contexts external to the xronos program.

    Physical events may be used to schedule new events from an external context
    outside of the scope of the reactor program. These could be external event
    handlers that respond to sensor inputs. Physical events do not provide
    direct access to their values. A :attr:`reaction` may declare a
    :class:`Trigger` to access the value associated with an
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

    def trigger(self, value: T) -> None:
        """Create a new event with an automatically assigned timestamp.

        Args:
            value: A value to be associated with the newly created event.
        """
        self._schedule(value, datetime.timedelta(0))

    def _get(self) -> T:
        return cast(T, super()._get())


class PhysicalEventDeclaration(ElementDescriptor[PhysicalEvent[T]]):
    """A declaration for a :class:`PhysicalEvent[T]<PhysicalEvent>`.

    Args:
        attributes(optional): A dict of attributes characterizing the physical event.

    Type Parameters:
        T(optional): The type of values carried by the physical event.
    """

    def __init__(self, attributes: AttributeMap | None = None) -> None:
        super().__init__(
            PhysicalEvent, frame_info=_get_calling_stack_frame(), attributes=attributes
        )


class Metric(runtime.Metric, AttributeMixin):
    """Allows recording values to an external data base from reaction handlers.

    This class is not intended to be instantiated directly. Use
    :class:`MetricDeclaration` instead.
    """

    @property
    def name(self) -> str:
        """Name of the metric (read-only)."""
        return self._name

    @property
    def fqn(self) -> str:
        """Fully qualified name of the metric (read-only)."""
        return self._fqn

    @property
    def description(self) -> str:
        """Description of the metric (read-only)."""
        return self._description

    @property
    def unit(self) -> str:
        """Unit of the metric (read-only)."""
        return self._unit


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
        super().__init__(
            lambda name, container: Metric(
                name,
                container,
                container.environment._metric_data_logger_provider,
                description,
                unit,
            ),
            frame_info=_get_calling_stack_frame(),
            attributes=attributes,
        )


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
            super().__init__(workers=workers, fast=fast, timeout=timeout)
        else:
            super().__init__(workers=workers, fast=fast)

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
        checked_class = Reactor._checked_cast_to_subclass(reactor_class)
        reactor = checked_class._create_instance(
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

    @deprecated("Use enable_telemetry() instead")
    def enable_tracing(
        self,
        application_name: str = "xronos",
        endpoint: str = "localhost:4317",
    ) -> None:
        """
        .. deprecated:: 0.3.0
           Use :func:`enable_telemetry` instead.
        """  # noqa: D205, D212
        self.enable_telemetry(application_name, endpoint)

    def enable_telemetry(
        self,
        application_name: str = "xronos",
        endpoint: str = "localhost:4317",
    ) -> None:
        """Enable the collection of telemetry data during program execution.

        See :ref:`telemetry` and  :ref:`dashboard` for more information on
        producing, collecting and visualizing telemetry data.

        Args:
            application_name: The name of the application as it should appear
                in the telemetry metadata.
            endpoint: The network endpoint to send telemetry data to. This is
                typically port 4137 on the host running the :ref:`dashboard`.
        """
        backend = runtime.OtelTelemetryBackend(
            self._attribute_manager,
            application_name,
            endpoint,
            platform.node(),
            os.getpid(),
        )
        self._set_telemetry_backend(cast(TelemetryBackend, backend))

    def request_shutdown(self) -> None:
        """Request the termination of the currently running reactor program.

        Terminates a program started with :func:`execute` at the next
        convenience. This triggers :attr:`~xronos.Reactor.shutdown` after
        completing all currently active reactions, and stops program execution after
        processing all reactions triggered by :attr:`~xronos.Reactor.shutdown`.
        """
        return super().request_shutdown()


class Reactor(runtime.Reactor, AttributeMixin):
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
            :func:`get_time` and the time at which the program started.
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
        checked_class = Reactor._checked_cast_to_subclass(reactor_class)
        reactor = checked_class._create_instance(
            name, self, _get_calling_stack_frame(), *args, **kwargs
        )
        # keep a reference to the created reactor
        self.__reactors.append(reactor)
        return reactor


TypedEventSource: TypeAlias = Port[T] | ProgrammableTimer[T] | PhysicalEvent[T]

ValuelessEventSource: TypeAlias = PeriodicTimer | Startup | Shutdown

GenericEventSource: TypeAlias = TypedEventSource[T] | ValuelessEventSource


class _Dependency(Generic[T]):
    def __init__(self, element: GenericEventSource[T]) -> None:
        self._element = element

    def is_present(self) -> bool:
        """Check if the referenced element has triggered."""
        return self._element._is_present


class AbsentError(Exception):
    """Indicates an attempt to read a value that is absent.

    See :func:`Trigger.get`.
    """

    pass


class _ReadableEvent(Generic[T], _Dependency[T]):
    def __init__(self, element: GenericEventSource[T]) -> None:
        super().__init__(element)

    def get(self) -> T:
        """Get the current value of the referenced reactor element.

        Raises:
            AbsentException: If called and :func:`is_present` returns ``False``.
        """
        if not self.is_present():
            raise AbsentError(
                f"Tried to read a value from {self._element.fqn},"
                " but there is no present value."
            )

        if isinstance(
            self._element,
            (runtime.Timer, runtime.Startup, runtime.Shutdown),
        ):
            return cast(T, None)
        return self._element._get()


class Trigger(Generic[T], _ReadableEvent[T]):
    """Access to a reactor element that a reaction is triggered by and may read.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_trigger` instead.
    """

    pass


class PortEffect(Generic[T], _ReadableEvent[T]):
    """Access to a port that a reaction may write to.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, element: Port[T]) -> None:
        super().__init__(element)
        # overwrite so that type checker knows element is a Port[T]
        self._element = element

    def set(self, value: T) -> None:
        """Set the port value and send a message to connected ports.

        Can be called multiple times, but at each time at most one value is
        sent to connected ports. When called repeatedly at a given timestamp, the
        previous value is overwritten.

        Args:
            value: The value to be written to the referenced port.
        """
        self._element._set(value)


class ProgrammableTimerEffect(Generic[T], _ReadableEvent[T]):
    """Access to a programmable timer that a reaction may schedule events with.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, element: ProgrammableTimer[T]) -> None:
        super().__init__(element)
        # overwrite so that type checker knows element is an ProgrammableTimer[T]
        self._element = element

    def schedule(
        self, value: T, delay: datetime.timedelta = datetime.timedelta(0)
    ) -> None:
        """Schedule a future timer event.

        Args:
            value: The value to be associated with the event occurrence.
            delay: The time to wait until the new event is processed.
        """
        self._element._schedule(value, delay)


class MetricEffect:
    """Access to a metric that can be recorded.

    This class is not intended to be instantiated directly. Use
    :func:`~xronos.ReactionInterface.add_effect` instead.
    """

    def __init__(self, metric: Metric) -> None:
        self._metric = metric

    def record(self, value: int | float) -> None:
        """Record a value.

        Args:
            value: The value to be recorded.
        """
        self._metric._record(value)


class ReactionInterface:
    """Helper class for defining the interfaces of a reaction.

    This class is not intended to be instantiated directly. An instance of this
    class is passed automatically to any method decorated with
    {attr}`~xronos.reaction`.
    """

    def __init__(self) -> None:
        self._triggers = set[GenericEventSource[Any]]()
        self._effects = set[Port[Any] | ProgrammableTimer[Any]]()

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
    def add_effect(self, target: Port[T]) -> PortEffect[T]: ...

    @overload
    def add_effect(
        self, target: ProgrammableTimer[T]
    ) -> ProgrammableTimerEffect[T]: ...

    @overload
    def add_effect(self, target: Metric) -> MetricEffect: ...

    def add_effect(
        self, target: Port[T] | ProgrammableTimer[T] | Metric
    ) -> PortEffect[T] | ProgrammableTimerEffect[T] | MetricEffect:
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
            self._effects.add(target)
            return PortEffect(target)
        elif isinstance(target, ProgrammableTimer):
            self._effects.add(target)
            return ProgrammableTimerEffect(target)
        else:
            return MetricEffect(target)


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
        self._declaration_func = declaration_func
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
        handler: Callable[[], None] = self._declaration_func(reactor, interface)
        if not callable(handler):
            raise RuntimeError(
                "Reaction declaration function did not return a callable"
            )
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
            a reaction handler. Failing to return a handler will result in an
            exception when the reactor containing the reaction is initialized.
    """
    return ReactionDescriptor[R](declaration, _get_calling_stack_frame())

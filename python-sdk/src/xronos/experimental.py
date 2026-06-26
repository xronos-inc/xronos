# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# Ignore access to protected/private members. These subclasses use the
# protected accessors of Reactor (_context, _register_dynamic_element) to
# build and register elements, mirroring how _core.py implements elements.
# pyright: reportPrivateUsage = false

"""Experimental API for creating reactor elements dynamically.

.. warning::
    This module is **experimental**. Its API may change or be removed in any
    release without notice.

The declarative API (assigning ``<Element>Declaration`` descriptors as class
attributes) requires every element to be statically known when the reactor
class is defined. The classes in this module instead let a reactor create
elements **dynamically** at runtime by instantiating them directly with an
explicit name and parent reactor -- analogous to
:meth:`~xronos.Reactor.create_reactor` for nested reactors, and to the C++ SDK.

Elements are typically created inside a reactor's ``__init__`` (after calling
``super().__init__()``)::

    import xronos
    import xronos.experimental


    class Wrapper(xronos.Reactor):
        def __init__(self, port_names: list[str]) -> None:
            super().__init__()
            self.inputs = {
                name: xronos.experimental.InputPort[int](name, self)
                for name in port_names
            }

The created element is automatically registered with ``parent`` (keeping the
Python wrapper alive), exactly like a declared element. Names must be unique
within the parent; a duplicate raises :class:`~xronos.DuplicateNameError`.
"""

import datetime
from typing import TypeVar

import xronos._cpp_sdk as sdk
from xronos import _core

__all__ = [
    "InputPort",
    "Metric",
    "OutputPort",
    "PeriodicTimer",
    "PhysicalEvent",
    "ProgrammableTimer",
]

T = TypeVar("T")


class InputPort(_core.InputPort[T]):
    """Dynamically created :class:`~xronos.InputPort`.

    Args:
        name: The name of the port. Must be unique within ``parent``.
        parent: The reactor that contains the port.
        attributes: A dict of attributes characterizing the port (optional).

    Type Parameters:
        T: The value type associated with messages.
    """

    def __init__(
        self,
        name: str,
        parent: _core.Reactor,
        attributes: _core.AttributeMap | None = None,
    ) -> None:
        context = parent._context(_core.get_source_location())
        super().__init__(sdk.InputPort(name, context))
        parent._register_dynamic_element(name, self)
        if attributes:
            self.add_attributes(attributes)


class OutputPort(_core.OutputPort[T]):
    """Dynamically created :class:`~xronos.OutputPort`.

    Args:
        name: The name of the port. Must be unique within ``parent``.
        parent: The reactor that contains the port.
        attributes: A dict of attributes characterizing the port (optional).

    Type Parameters:
        T: The value type associated with messages.
    """

    def __init__(
        self,
        name: str,
        parent: _core.Reactor,
        attributes: _core.AttributeMap | None = None,
    ) -> None:
        context = parent._context(_core.get_source_location())
        super().__init__(sdk.OutputPort(name, context))
        parent._register_dynamic_element(name, self)
        if attributes:
            self.add_attributes(attributes)


class PeriodicTimer(_core.PeriodicTimer):
    """Dynamically created :class:`~xronos.PeriodicTimer`.

    Args:
        name: The name of the timer. Must be unique within ``parent``.
        parent: The reactor that contains the timer.
        period: The delay in between two events emitted by the timer (optional).
        offset: The delay between the :attr:`~xronos.Reactor.startup` event and
            the first event emitted by the timer (optional).
        attributes: A dict of attributes characterizing the timer (optional).
    """

    def __init__(
        self,
        name: str,
        parent: _core.Reactor,
        period: datetime.timedelta = datetime.timedelta(0),
        offset: datetime.timedelta = datetime.timedelta(0),
        attributes: _core.AttributeMap | None = None,
    ) -> None:
        context = parent._context(_core.get_source_location())
        super().__init__(sdk.PeriodicTimer(name, context, period, offset))
        parent._register_dynamic_element(name, self)
        if attributes:
            self.add_attributes(attributes)


class ProgrammableTimer(_core.ProgrammableTimer[T]):
    """Dynamically created :class:`~xronos.ProgrammableTimer`.

    Args:
        name: The name of the programmable timer. Must be unique within
            ``parent``.
        parent: The reactor that contains the programmable timer.
        attributes: A dict of attributes characterizing the programmable timer
            (optional).

    Type Parameters:
        T: The value type associated with events emitted by the programmable
           timer.
    """

    def __init__(
        self,
        name: str,
        parent: _core.Reactor,
        attributes: _core.AttributeMap | None = None,
    ) -> None:
        context = parent._context(_core.get_source_location())
        super().__init__(sdk.ProgrammableTimer(name, context))
        parent._register_dynamic_element(name, self)
        if attributes:
            self.add_attributes(attributes)


class PhysicalEvent(_core.PhysicalEvent[T]):
    """Dynamically created :class:`~xronos.PhysicalEvent`.

    Args:
        name: The name of the physical event. Must be unique within ``parent``.
        parent: The reactor that contains the physical event.
        attributes: A dict of attributes characterizing the physical event
            (optional).

    Type Parameters:
        T: The type of values carried by emitted events.
    """

    def __init__(
        self,
        name: str,
        parent: _core.Reactor,
        attributes: _core.AttributeMap | None = None,
    ) -> None:
        context = parent._context(_core.get_source_location())
        super().__init__(sdk.PhysicalEvent(name, context))
        parent._register_dynamic_element(name, self)
        if attributes:
            self.add_attributes(attributes)


class Metric(_core.Metric):
    """Dynamically created :class:`~xronos.Metric`.

    Args:
        name: The name of the metric. Must be unique within ``parent``.
        parent: The reactor that contains the metric.
        description: A description of the metric.
        unit: The unit of the metric (optional).
        attributes: A dict of attributes characterizing the metric (optional).
    """

    def __init__(
        self,
        name: str,
        parent: _core.Reactor,
        description: str,
        unit: str | None = None,
        attributes: _core.AttributeMap | None = None,
    ) -> None:
        context = parent._context(_core.get_source_location())
        super().__init__(sdk.Metric(name, context, description, unit or ""))
        parent._register_dynamic_element(name, self)
        if attributes:
            self.add_attributes(attributes)

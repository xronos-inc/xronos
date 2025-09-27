# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable

class ValidationError(Exception):
    def __init__(self, message: str) -> None: ...

class SourceLocation:
    def __init__(
        self,
        file_: str,
        function: str,
        start_line: int,
        end_line: int,
        start_column: int,
        end_column: int,
    ) -> None: ...

class EnvironmentContext:
    pass

class ReactorContext:
    pass

class BaseEnvironment:
    pass

class Environment(BaseEnvironment):
    def __init__(
        self,
        workers: int,
        fast: bool,
        render_reactor_graph: bool,
        timeout: datetime.timedelta = ...,
    ) -> None: ...
    def execute(self) -> None: ...
    def enable_telemetry(self, application_name: str, endpoint: str) -> None: ...
    def connect(
        self, from_: InputPort | OutputPort, to: InputPort | OutputPort
    ) -> None: ...
    def connect_delayed(
        self,
        from_: InputPort | OutputPort,
        to: InputPort | OutputPort,
        delay: datetime.timedelta,
    ) -> None: ...
    def context(self, source_location: SourceLocation) -> EnvironmentContext: ...

class Element:
    @property
    def name(self) -> str: ...
    @property
    def fqn(self) -> str: ...
    def add_attribute(self, key: str, value: str | bool | int | float) -> bool: ...
    def add_attributes(
        self, attributes: dict[str, str | bool | int | float]
    ) -> bool: ...

class BaseReactor(Element):
    pass

class Reactor(BaseReactor):
    def __init__(
        self,
        name: str,
        context: ReactorContext | EnvironmentContext,
        assemble_callback: Callable[[], None],
    ) -> None: ...
    def get_lag(self) -> datetime.timedelta: ...
    def get_time(self) -> datetime.datetime: ...
    def get_time_since_startup(self) -> datetime.timedelta: ...
    @property
    def startup(self) -> Startup: ...
    @property
    def shutdown(self) -> Shutdown: ...
    def add_reaction(
        self,
        name: str,
        soure_location: SourceLocation,
    ) -> Reaction: ...
    def connect(
        self, from_: InputPort | OutputPort, to: InputPort | OutputPort
    ) -> None: ...
    def connect_delayed(
        self,
        from_: InputPort | OutputPort,
        to: InputPort | OutputPort,
        delay: datetime.timedelta,
    ) -> None: ...
    def context(self, source_location: SourceLocation) -> ReactorContext: ...

class Startup(Element):
    pass

class Shutdown(Element):
    pass

class PeriodicTimer(Element):
    def __init__(
        self,
        name: str,
        context: ReactorContext,
        period: datetime.timedelta,
        offset: datetime.timedelta,
    ) -> None: ...
    def period(self) -> datetime.timedelta: ...
    def offset(self) -> datetime.timedelta: ...
    def set_period(self, period: datetime.timedelta) -> None: ...
    def set_offset(self, offset: datetime.timedelta) -> None: ...

class InputPort(Element):
    def __init__(self, name: str, context: ReactorContext) -> None: ...

class OutputPort(Element):
    def __init__(self, name: str, context: ReactorContext) -> None: ...

class ProgrammableTimer(Element):
    def __init__(self, name: str, context: ReactorContext) -> None: ...

class PhysicalEvent(Element):
    def __init__(self, name: str, context: ReactorContext) -> None: ...
    def trigger(self, value: object) -> None: ...

class Metric(Element):
    def __init__(
        self, name: str, context: ReactorContext, description: str, unit: str
    ) -> None: ...
    @property
    def description(self) -> str: ...
    @property
    def unit(self) -> str: ...

class ReactionContext:
    pass

class BaseReaction(Element):
    pass

class Reaction(BaseReaction):
    def set_handler(self, handler: Callable[[], None]) -> None: ...
    def context(self) -> ReactionContext: ...

class VoidTrigger:
    def __init__(
        self, trigger: PeriodicTimer | Startup | Shutdown, context: ReactionContext
    ) -> None: ...
    def is_present(self) -> bool: ...

class Trigger:
    def __init__(
        self,
        trigger: InputPort | OutputPort | PhysicalEvent | ProgrammableTimer,
        context: ReactionContext,
    ) -> None: ...
    def is_present(self) -> bool: ...
    def get(self) -> object: ...

class PortEffect:
    def __init__(
        self, port: InputPort | OutputPort, context: ReactionContext
    ) -> None: ...
    def set(self, value: object) -> None: ...
    def is_present(self) -> bool: ...
    def get(self) -> object: ...

class ProgrammableTimerEffect:
    def __init__(
        self, programmable_timer: ProgrammableTimer, context: ReactionContext
    ) -> None: ...
    def schedule(self, value: object, delay: datetime.timedelta) -> None: ...

class MetricEffect:
    def __init__(self, metric: Metric, context: ReactionContext) -> None: ...
    def record(self, value: int | float) -> None: ...

class ShutdownEffect:
    def __init__(self, shutdown: Shutdown, context: ReactionContext) -> None: ...
    def trigger_shutdown(self) -> None: ...

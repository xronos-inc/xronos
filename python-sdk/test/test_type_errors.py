# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# type: ignore

import datetime
import re

import pytest

import xronos
import xronos.experimental


def raises_type_error(message):
    return pytest.raises(TypeError, match=re.escape(message))


def execute_single(reactor_class):
    env = xronos.Environment(fast=True)
    env.create_reactor("reactor", reactor_class)
    env.execute()


class Empty(xronos.Reactor):
    pass


class Source(xronos.Reactor):
    output = xronos.OutputPortDeclaration()


class Sink(xronos.Reactor):
    input_ = xronos.InputPortDeclaration()
    metric = xronos.MetricDeclaration("a metric")


class WithTimer(xronos.Reactor):
    timer = xronos.PeriodicTimerDeclaration()


@pytest.mark.parametrize(
    ("declare", "message"),
    [
        (
            lambda: xronos.PeriodicTimerDeclaration(period=5),
            "The argument 'period' to PeriodicTimerDeclaration() must be a "
            "timedelta, but got an int.",
        ),
        (
            lambda: xronos.PeriodicTimerDeclaration(offset="never"),
            "The argument 'offset' to PeriodicTimerDeclaration() must be a "
            "timedelta, but got a str.",
        ),
        (
            lambda: xronos.MetricDeclaration(42),
            "The argument 'description' to MetricDeclaration() must be a str, "
            "but got an int.",
        ),
        (
            lambda: xronos.MetricDeclaration("a metric", unit=42),
            "The argument 'unit' to MetricDeclaration() must be a str, but got an int.",
        ),
        (
            lambda: xronos.InputPortDeclaration(attributes=[("key", "value")]),
            "The argument 'attributes' to InputPortDeclaration() must be a "
            "dict, but got a list.",
        ),
        (
            lambda: xronos.OutputPortDeclaration(attributes={1: "value"}),
            "Each key of the argument 'attributes' to OutputPortDeclaration() "
            "must be a str, but got an int.",
        ),
        (
            lambda: xronos.ProgrammableTimerDeclaration(attributes={"key": None}),
            "Each value of the argument 'attributes' to "
            "ProgrammableTimerDeclaration() must be a str, bool, int, or "
            "float, but got a NoneType.",
        ),
        (
            lambda: xronos.PhysicalEventDeclaration(attributes={"key": [1]}),
            "Each value of the argument 'attributes' to "
            "PhysicalEventDeclaration() must be a str, bool, int, or float, "
            "but got a list.",
        ),
        (
            lambda: xronos.MetricDeclaration("a metric", attributes="key"),
            "The argument 'attributes' to MetricDeclaration() must be a dict, "
            "but got a str.",
        ),
    ],
)
def test_declaration_constructors(declare, message):
    with raises_type_error(message):
        declare()


@pytest.mark.parametrize(
    ("construct", "message"),
    [
        (
            lambda: xronos.experimental.InputPort("port", 42),
            "The argument 'parent' to InputPort() must be a Reactor, but got an int.",
        ),
        (
            lambda: xronos.experimental.OutputPort("port", None),
            "The argument 'parent' to OutputPort() must be a Reactor, "
            "but got a NoneType.",
        ),
        (
            lambda: xronos.experimental.PeriodicTimer("timer", "reactor"),
            "The argument 'parent' to PeriodicTimer() must be a Reactor, "
            "but got a str.",
        ),
        (
            lambda: xronos.experimental.ProgrammableTimer("timer", 42),
            "The argument 'parent' to ProgrammableTimer() must be a Reactor, "
            "but got an int.",
        ),
        (
            lambda: xronos.experimental.PhysicalEvent("event", 42),
            "The argument 'parent' to PhysicalEvent() must be a Reactor, "
            "but got an int.",
        ),
        (
            lambda: xronos.experimental.Metric("metric", 42, "a metric"),
            "The argument 'parent' to Metric() must be a Reactor, but got an int.",
        ),
        (
            lambda: xronos.experimental.InputPort(42, 42),
            "The argument 'name' to InputPort() must be a str, but got an int.",
        ),
    ],
)
def test_experimental_constructors(construct, message):
    with raises_type_error(message):
        construct()


@pytest.mark.parametrize(
    ("build", "message"),
    [
        (
            lambda reactor: xronos.experimental.PeriodicTimer(
                "timer", reactor, period=5
            ),
            "The argument 'period' to PeriodicTimer() must be a timedelta, "
            "but got an int.",
        ),
        (
            lambda reactor: xronos.experimental.PeriodicTimer(
                "timer", reactor, offset="never"
            ),
            "The argument 'offset' to PeriodicTimer() must be a timedelta, "
            "but got a str.",
        ),
        (
            lambda reactor: xronos.experimental.Metric("metric", reactor, 42),
            "The argument 'description' to Metric() must be a str, but got an int.",
        ),
        (
            lambda reactor: xronos.experimental.Metric(
                "metric", reactor, "a metric", unit=42
            ),
            "The argument 'unit' to Metric() must be a str, but got an int.",
        ),
        (
            lambda reactor: xronos.experimental.OutputPort(
                "port", reactor, attributes={"key": None}
            ),
            "Each value of the argument 'attributes' to OutputPort() must be "
            "a str, bool, int, or float, but got a NoneType.",
        ),
    ],
)
def test_experimental_constructors_with_parent(build, message):
    class Dynamic(xronos.Reactor):
        def __init__(self):
            super().__init__()
            build(self)

    env = xronos.Environment(fast=True)
    with raises_type_error(message):
        env.create_reactor("dynamic", Dynamic)


def test_environment_timeout():
    with raises_type_error(
        "The argument 'timeout' to Environment() must be a timedelta, but got an int."
    ):
        xronos.Environment(timeout=5)


def test_environment_connect_from_wrong_element():
    env = xronos.Environment(fast=True)
    env.create_reactor("source", Source)
    sink = env.create_reactor("sink", Sink)
    with raises_type_error(
        "The argument 'from_' to Environment.connect() must be an InputPort "
        "or OutputPort, but got a Metric."
    ):
        env.connect(sink.metric, sink.input_)


def test_environment_connect_to_not_an_element():
    env = xronos.Environment(fast=True)
    source = env.create_reactor("source", Source)
    with raises_type_error(
        "The argument 'to' to Environment.connect() must be an InputPort "
        "or OutputPort, but got an int."
    ):
        env.connect(source.output, 42)


def test_environment_connect_delay_wrong_type():
    env = xronos.Environment(fast=True)
    source = env.create_reactor("source", Source)
    sink = env.create_reactor("sink", Sink)
    with raises_type_error(
        "The argument 'delay' to Environment.connect() must be a timedelta, "
        "but got a float."
    ):
        env.connect(source.output, sink.input_, delay=1.0)


class BadConnectComposite(xronos.Reactor):
    def __init__(self):
        super().__init__()
        source = self.create_reactor("source", Source)
        self.connect(source.output, source)


def test_reactor_connect_to_wrong_element():
    env = xronos.Environment(fast=True)
    with raises_type_error(
        "The argument 'to' to Reactor.connect() must be an InputPort "
        "or OutputPort, but got a Source."
    ):
        env.create_reactor("composite", BadConnectComposite)


def test_environment_create_reactor_name_wrong_type():
    env = xronos.Environment(fast=True)
    with raises_type_error(
        "The argument 'name' to Environment.create_reactor() must be a str, "
        "but got an int."
    ):
        env.create_reactor(42, Empty)


def test_environment_create_reactor_not_a_class():
    env = xronos.Environment(fast=True)
    with raises_type_error(
        "The argument 'reactor_class' to Environment.create_reactor() must "
        "be a subclass of Reactor, but got an int."
    ):
        env.create_reactor("reactor", 42)


def test_environment_create_reactor_wrong_class():
    env = xronos.Environment(fast=True)
    with raises_type_error(
        "The argument 'reactor_class' to Environment.create_reactor() must "
        "be a subclass of Reactor, but got the class int."
    ):
        env.create_reactor("reactor", int)


class BadCreateComposite(xronos.Reactor):
    def __init__(self):
        super().__init__()
        self.create_reactor("inner", 42)


def test_reactor_create_reactor_not_a_class():
    env = xronos.Environment(fast=True)
    with raises_type_error(
        "The argument 'reactor_class' to Reactor.create_reactor() must "
        "be a subclass of Reactor, but got an int."
    ):
        env.create_reactor("composite", BadCreateComposite)


class BadTriggerValue(xronos.Reactor):
    @xronos.reaction
    def bad(self, ctx):
        ctx.add_trigger(42)
        return lambda: None


def test_add_trigger_not_an_element():
    # The reaction declaration may run at create_reactor() or at execute(),
    # so the block deliberately spans both.
    with raises_type_error(
        "The argument 'event_source' to ReactionContext.add_trigger() must "
        "be an EventSource, but got an int."
    ):
        execute_single(BadTriggerValue)


class BadTriggerKind(xronos.Reactor):
    metric = xronos.MetricDeclaration("a metric")

    @xronos.reaction
    def bad(self, ctx):
        ctx.add_trigger(self.metric)
        return lambda: None


def test_add_trigger_wrong_element_kind():
    with raises_type_error(
        "The argument 'event_source' to ReactionContext.add_trigger() must "
        "be an EventSource, but got a Metric."
    ):
        execute_single(BadTriggerKind)


class BadEffectValue(xronos.Reactor):
    @xronos.reaction
    def bad(self, ctx):
        ctx.add_trigger(self.startup)
        ctx.add_effect(42)
        return lambda: None


def test_add_effect_not_an_element():
    with raises_type_error(
        "The argument 'target' to ReactionContext.add_effect() must be an "
        "InputPort, OutputPort, ProgrammableTimer, Metric, or Shutdown, "
        "but got an int."
    ):
        execute_single(BadEffectValue)


class BadEffectKind(xronos.Reactor):
    @xronos.reaction
    def bad(self, ctx):
        ctx.add_trigger(self.startup)
        ctx.add_effect(self.startup)
        return lambda: None


def test_add_effect_wrong_element_kind():
    with raises_type_error(
        "The argument 'target' to ReactionContext.add_effect() must be an "
        "InputPort, OutputPort, ProgrammableTimer, Metric, or Shutdown, "
        "but got a Startup."
    ):
        execute_single(BadEffectKind)


class BadRecord(xronos.Reactor):
    metric = xronos.MetricDeclaration("a metric")

    @xronos.reaction
    def bad(self, ctx):
        ctx.add_trigger(self.startup)
        metric = ctx.add_effect(self.metric)
        return lambda: metric.record("high")


def test_metric_record_wrong_type():
    with raises_type_error(
        "The argument 'value' to MetricEffect.record() must be an int or "
        "float, but got a str."
    ):
        execute_single(BadRecord)


class BadSchedule(xronos.Reactor):
    timer = xronos.ProgrammableTimerDeclaration()

    @xronos.reaction
    def bad(self, ctx):
        ctx.add_trigger(self.startup)
        timer = ctx.add_effect(self.timer)
        return lambda: timer.schedule(None, delay=5)


def test_schedule_delay_wrong_type():
    with raises_type_error(
        "The argument 'delay' to ProgrammableTimerEffect.schedule() must be "
        "a timedelta, but got an int."
    ):
        execute_single(BadSchedule)


def test_add_attribute_key_wrong_type():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", Empty)
    with raises_type_error(
        "The argument 'key' to Element.add_attribute() must be a str, but got an int."
    ):
        reactor.add_attribute(42, "value")


def test_add_attribute_value_wrong_type():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", Empty)
    with raises_type_error(
        "The argument 'value' to Element.add_attribute() must be a str, "
        "bool, int, or float, but got a NoneType."
    ):
        reactor.add_attribute("key", None)


def test_add_attributes_not_a_dict():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", Empty)
    with raises_type_error(
        "The argument 'attributes' to Element.add_attributes() must be a "
        "dict, but got a list."
    ):
        reactor.add_attributes([("key", "value")])


def test_add_attributes_key_wrong_type():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", Empty)
    with raises_type_error(
        "Each key of the argument 'attributes' to Element.add_attributes() "
        "must be a str, but got an int."
    ):
        reactor.add_attributes({42: "value"})


def test_add_attributes_value_wrong_type():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", Empty)
    with raises_type_error(
        "Each value of the argument 'attributes' to Element.add_attributes() "
        "must be a str, bool, int, or float, but got a dict."
    ):
        reactor.add_attributes({"key": {}})


def test_periodic_timer_period_setter():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", WithTimer)
    with raises_type_error(
        "The value assigned to PeriodicTimer.period must be a timedelta, "
        "but got an int."
    ):
        reactor.timer.period = 5


def test_periodic_timer_offset_setter():
    env = xronos.Environment(fast=True)
    reactor = env.create_reactor("reactor", WithTimer)
    with raises_type_error(
        "The value assigned to PeriodicTimer.offset must be a timedelta, but got a str."
    ):
        reactor.timer.offset = "never"


def test_reaction_with_deadline_wrong_type():
    with raises_type_error(
        "The argument 'deadline' to reaction_with_deadline() must be a "
        "timedelta, but got an int."
    ):
        xronos.reaction_with_deadline(deadline=5)


class ValidSource(xronos.Reactor):
    output = xronos.OutputPortDeclaration(attributes={"role": "source"})
    timer = xronos.PeriodicTimerDeclaration(
        period=datetime.timedelta(milliseconds=1),
        offset=datetime.timedelta(0),
        attributes={"kind": "periodic", "enabled": True, "count": 1, "scale": 0.5},
    )

    @xronos.reaction
    def emit(self, ctx):
        ctx.add_trigger(self.timer)
        output = ctx.add_effect(self.output)
        return lambda: output.set(42)


class ValidSink(xronos.Reactor):
    input_ = xronos.InputPortDeclaration(attributes={"role": "sink"})
    metric = xronos.MetricDeclaration("count of received messages", "messages")
    programmable = xronos.ProgrammableTimerDeclaration()
    event = xronos.PhysicalEventDeclaration()

    @xronos.reaction
    def receive(self, ctx):
        trigger = ctx.add_trigger(self.input_)
        metric = ctx.add_effect(self.metric)
        programmable = ctx.add_effect(self.programmable)
        shutdown = ctx.add_effect(self.shutdown)

        def handler():
            metric.record(trigger.get())
            metric.record(0.5)
            metric.record(True)
            programmable.schedule(None, delay=datetime.timedelta(milliseconds=1))
            shutdown.trigger_shutdown()

        return handler


def test_valid_program():
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=1))
    source = env.create_reactor("source", ValidSource)
    sink = env.create_reactor("sink", ValidSink)
    env.connect(source.output, sink.input_, delay=datetime.timedelta(milliseconds=1))
    source.timer.period = datetime.timedelta(milliseconds=2)
    source.timer.offset = datetime.timedelta(milliseconds=1)
    source.add_attribute("key", "value")
    sink.add_attributes({"a": 1, "b": True, "c": 0.1, "d": "x"})
    env.execute()


class ValidComposite(xronos.Reactor):
    def __init__(self):
        super().__init__()
        source = self.create_reactor("source", ValidSource)
        sink = self.create_reactor("sink", ValidSink)
        self.connect(source.output, sink.input_)


def test_valid_composite():
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=1))
    env.create_reactor("composite", ValidComposite)
    env.execute()


class ValidDynamic(xronos.Reactor):
    def __init__(self):
        super().__init__()
        self.input_ = xronos.experimental.InputPort("input", self)
        self.output = xronos.experimental.OutputPort(
            "output", self, attributes={"role": "source"}
        )
        self.timer = xronos.experimental.PeriodicTimer(
            "timer",
            self,
            period=datetime.timedelta(milliseconds=1),
            offset=datetime.timedelta(0),
        )
        self.programmable = xronos.experimental.ProgrammableTimer("programmable", self)
        self.event = xronos.experimental.PhysicalEvent("event", self)
        self.metric = xronos.experimental.Metric("metric", self, "a metric", "units")


def test_valid_dynamic_elements():
    env = xronos.Environment(fast=True, timeout=datetime.timedelta(seconds=1))
    env.create_reactor("dynamic", ValidDynamic)
    env.execute()

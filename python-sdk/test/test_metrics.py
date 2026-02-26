# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

from typing import Callable

import pytest

import xronos


class HelloNested(xronos.Reactor):
    hello_metric = xronos.MetricDeclaration(
        "nested description",
        None,
        attributes={"attr0": "hello_nested", "attr1": "world_nested"},
    )

    def __init__(self) -> None:
        super().__init__()
        self.add_attributes({"nested_attr": "nested_value"})

    @xronos.reaction
    def hello(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        hello_metric_writable = interface.add_effect(self.hello_metric)

        def handler() -> None:
            self.add_attribute("dynamic_attr", "dynamic_value")
            self.add_attribute("attr0", "attr0_value2")
            hello_metric_writable.record(41.93)
            # attributes in ancestor reactors can be overridden in the current reactor
            self.add_attribute("hello_reactor_attr", "hello_reactor_value2")
            # attributes in the current reactor cannot be added again
            with pytest.raises(KeyError):
                self.add_attribute("nested_attr", "nested_attr2")
            with pytest.raises(KeyError):
                self.add_attribute("dynamic_attr", "dynamic_attr2")
            print(f"{self.fqn} says hello!")

        return handler


class Hello(xronos.Reactor):
    hello_metric = xronos.MetricDeclaration(
        "hello description",
        None,
        attributes={"attr0": "hello", "attr1": "world", "attr2": "outer"},
    )

    def __init__(self) -> None:
        super().__init__()
        self.add_attribute("hello_reactor_attr", "hello_reactor_value")
        self.nested = self.create_reactor("nested", HelloNested)

    @xronos.reaction
    def hello(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        hello_metric_writable = interface.add_effect(self.hello_metric)

        def handler() -> None:
            hello_metric_writable.record(42.93)
            print(f"{self.fqn} says hello!")
            assert self.hello_metric.description == "hello description"
            assert self.hello_metric.unit == ""

        return handler


def run(env: xronos.Environment) -> None:
    # passing this test involves calling telemetry APIs without errors
    env.create_reactor("hello", Hello)
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_metrics() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

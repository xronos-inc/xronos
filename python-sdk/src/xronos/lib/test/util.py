# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from collections.abc import Callable
from typing import Generic, TypeVar

import xronos

T = TypeVar("T")


class AssertList(xronos.Reactor, Generic[T]):
    """A reactor that asserts its input values against an expected list.

    It also terminates the execution when all the expected values have been received.
    """

    input_ = xronos.InputPortDeclaration[T]()

    def __init__(self, expected: list[T], debug: bool = False) -> None:
        super().__init__()
        self.expected = expected
        self.expected_iter = iter(expected)
        self.count = 0
        self.debug = debug

    @xronos.reaction
    def on_input(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        input_trigger = ctx.add_trigger(self.input_)
        shutdown_effect = ctx.add_effect(self.shutdown)

        def handler() -> None:
            value = input_trigger.get()
            if self.debug:
                print(f"{datetime.datetime.now()} - Received: {value}")
            expected_value = next(self.expected_iter)
            assert value == expected_value, (
                f"expected {expected_value!r}, got {value!r}"
            )
            self.count += 1

            # When all the expected values have been received, request a shutdown.
            if self.count == len(self.expected):
                shutdown_effect.trigger_shutdown()

        return handler

    @xronos.reaction
    def on_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.shutdown)

        def handler() -> None:
            assert self.count == len(self.expected), (
                f"received {self.count} of {len(self.expected)} values"
            )

        return handler


class AssertBytes(xronos.Reactor):
    """A reactor that asserts the concatenation of its input bytes.

    It terminates the execution once it has received as many bytes as expected. The
    bytes may arrive in any number of input events.
    """

    input_ = xronos.InputPortDeclaration[bytes]()

    def __init__(self, expected: bytes, debug: bool = False) -> None:
        super().__init__()
        self.expected = expected
        self.received = b""
        self.debug = debug

    @xronos.reaction
    def on_input(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        input_trigger = ctx.add_trigger(self.input_)
        shutdown_effect = ctx.add_effect(self.shutdown)

        def handler() -> None:
            value = input_trigger.get()
            if self.debug:
                print(f"{datetime.datetime.now()} - Received: {value}")
            self.received += value
            # Fail on the first wrong byte instead of waiting for a shutdown that a
            # short stream would never trigger.
            assert self.expected.startswith(self.received), (
                f"expected a prefix of {self.expected!r}, got {self.received!r}"
            )

            # When all the expected bytes have arrived, request a shutdown.
            if self.received == self.expected:
                shutdown_effect.trigger_shutdown()

        return handler

    @xronos.reaction
    def on_shutdown(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.shutdown)

        def handler() -> None:
            assert self.received == self.expected, (
                f"expected {self.expected!r}, got {self.received!r}"
            )

        return handler

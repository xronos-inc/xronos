# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
from typing import Callable, Generic, TypeVar

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
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_trigger = interface.add_trigger(self.input_)

        def handler() -> None:
            if self.debug:
                print(f"{datetime.datetime.now()} - Received: {input_trigger.get()}")
            assert input_trigger.get() == next(self.expected_iter)
            self.count += 1

            # When all the expected values have been received, request a shutdown.
            if self.count == len(self.expected):
                self.environment.request_shutdown()

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            assert self.count == len(self.expected)

        return handler

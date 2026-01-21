# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# type: ignore

import pytest

import xronos


class NoneHandler(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)


class NotCallableHandler(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        return ""


class FunctionHandlerWithPositionalArg(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        def handler(x):
            pass

        return handler


class FunctionHandlerWithRequiredKeywordArg(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        def handler(*, x):
            pass

        return handler


class FunctorWithArgs(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        class Handler:
            def __call__(self, x, y):
                pass

        return Handler()


def test_invalid_reaction_handler():
    for reactor_class in [
        NoneHandler,
        NotCallableHandler,
        FunctionHandlerWithRequiredKeywordArg,
        FunctionHandlerWithPositionalArg,
        FunctorWithArgs,
    ]:
        with pytest.raises(xronos.InvalidReactionHandler):
            env = xronos.Environment()
            env.create_reactor("test", reactor_class)
            env.execute()


class FunctionHandlerWithOptionalArgs(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        def handler(x=4, y=""):
            pass

        return handler


class FunctionHandlerWithWildcards(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        def handler(*args, **kwargs):
            pass

        return handler


class FunctionWithoutArgs(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        def handler():
            pass

        return handler


class FunctorWithoutArgs(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        class Handler:
            def __call__(self):
                pass

        return Handler()


class FunctorWithDefaultArgs(xronos.Reactor):
    @xronos.reaction
    def reaction(self, interface):
        interface.add_trigger(self.startup)

        class Handler:
            def __call__(self, x=1):
                pass

        return Handler()


def test_valid_reaction_handler():
    for reactor_class in [
        FunctionHandlerWithOptionalArgs,
        FunctionHandlerWithWildcards,
        FunctionWithoutArgs,
        FunctorWithoutArgs,
        FunctorWithDefaultArgs,
    ]:
        env = xronos.Environment()
        env.create_reactor("test", reactor_class)
        env.execute()

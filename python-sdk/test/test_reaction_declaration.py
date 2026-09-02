# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# type: ignore

import datetime
import sys

import pytest

import xronos

SIGNATURE_ERROR = "exactly two positional arguments"


def test_too_few_arguments():
    with pytest.raises(TypeError, match=SIGNATURE_ERROR):

        class TooFewArguments(xronos.Reactor):
            @xronos.reaction
            def reaction(self):
                pass


def test_too_many_arguments():
    with pytest.raises(TypeError, match=SIGNATURE_ERROR):

        class TooManyArguments(xronos.Reactor):
            @xronos.reaction
            def reaction(self, ctx, x, y):
                pass


def test_not_callable():
    with pytest.raises(TypeError, match="must be callable"):

        class NotCallable(xronos.Reactor):
            reaction = xronos.reaction(42)


def test_staticmethod():
    with pytest.raises(TypeError, match="staticmethod or classmethod"):

        class StaticMethod(xronos.Reactor):
            @xronos.reaction
            @staticmethod
            def reaction(reactor, ctx):
                pass


def test_classmethod():
    with pytest.raises(TypeError, match="staticmethod or classmethod"):

        class ClassMethod(xronos.Reactor):
            @xronos.reaction
            @classmethod
            def reaction(cls, ctx):
                pass


def test_staticmethod_outermost():
    with pytest.raises(TypeError, match="staticmethod or classmethod"):

        class StaticMethodOutermost(xronos.Reactor):
            @staticmethod
            @xronos.reaction
            def reaction(reactor, ctx):
                pass


def test_classmethod_outermost():
    with pytest.raises(TypeError, match="staticmethod or classmethod"):

        class ClassMethodOutermost(xronos.Reactor):
            @classmethod
            @xronos.reaction
            def reaction(cls, ctx):
                pass


def test_too_few_arguments_with_deadline():
    with pytest.raises(TypeError, match=SIGNATURE_ERROR):

        class TooFewArgumentsWithDeadline(xronos.Reactor):
            @xronos.reaction_with_deadline(deadline=datetime.timedelta(seconds=1))
            def reaction(self):
                pass


@pytest.mark.skipif(
    sys.version_info < (3, 12),
    reason="Python <= 3.11 wraps errors raised from __set_name__",
)
def test_reaction_in_non_reactor_class():
    with pytest.raises(TypeError, match=r"subclasses of xronos\.Reactor"):

        class NotAReactor:
            @xronos.reaction
            def reaction(self, ctx):
                return lambda: None


@pytest.mark.skipif(
    sys.version_info >= (3, 12),
    reason="Python >= 3.12 propagates errors raised from __set_name__ unwrapped",
)
def test_reaction_in_non_reactor_class_wrapped():
    # Python <= 3.11 wraps a TypeError raised from __set_name__ in a
    # RuntimeError with the original as __cause__.
    with pytest.raises(RuntimeError) as excinfo:

        class NotAReactor:
            @xronos.reaction
            def reaction(self, ctx):
                return lambda: None

    cause = excinfo.value.__cause__
    assert isinstance(cause, TypeError)
    assert "subclasses of xronos.Reactor" in str(cause)


class TwoArguments(xronos.Reactor):
    @xronos.reaction
    def reaction(self, ctx):
        ctx.add_trigger(self.startup)
        return lambda: None


class DefaultArgument(xronos.Reactor):
    @xronos.reaction
    def reaction(self, ctx, x=42):
        ctx.add_trigger(self.startup)
        return lambda: None


class StarArgs(xronos.Reactor):
    @xronos.reaction
    def reaction(*args):
        reactor, ctx = args
        ctx.add_trigger(reactor.startup)
        return lambda: None


class Kwargs(xronos.Reactor):
    @xronos.reaction
    def reaction(self, ctx, **kwargs):
        ctx.add_trigger(self.startup)
        return lambda: None


class KeywordOnlyWithDefault(xronos.Reactor):
    @xronos.reaction
    def reaction(self, ctx, *, x=42):
        ctx.add_trigger(self.startup)
        return lambda: None


class ClassDeclaration(xronos.Reactor):
    @xronos.reaction
    class reaction:
        def __init__(self, reactor, ctx):
            ctx.add_trigger(reactor.startup)

        def __call__(self):
            pass


class WithDeadline(xronos.Reactor):
    @xronos.reaction_with_deadline(deadline=datetime.timedelta(seconds=1))
    def reaction(self, ctx):
        ctx.add_trigger(self.startup)
        return lambda: None


def test_valid_reaction_declaration():
    for reactor_class in [
        TwoArguments,
        DefaultArgument,
        StarArgs,
        Kwargs,
        KeywordOnlyWithDefault,
        ClassDeclaration,
        WithDeadline,
    ]:
        env = xronos.Environment(fast=True)
        env.create_reactor("test", reactor_class)
        env.execute()

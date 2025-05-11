# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# type: ignore
# ruff: noqa: PLR2004

import inspect
from typing import Callable

import pytest

import xronos._core as core


def test_decorator():
    reaction = object()
    descriptor = core.reaction(reaction)
    assert isinstance(descriptor, core.ReactionDescriptor)
    assert descriptor._declaration_func is reaction


class Reactor(core.Reactor):
    pass


class TestReactionDecorator:
    @staticmethod
    def test_init():
        reaction = object()
        desc = core.ReactionDescriptor(reaction, inspect.stack()[0])
        assert desc._declaration_func is reaction
        assert desc._name is None
        assert desc._priority is None
        assert len(desc._instances) == 0

    @staticmethod
    def test_set_name():
        desc_foo = core.ReactionDescriptor(object(), list())

        with pytest.raises(AssertionError):
            desc_foo.name
        with pytest.raises(AssertionError):
            desc_foo.priority

        desc_foo.__set_name__(Reactor, "foo")
        assert desc_foo.name == "foo"
        assert desc_foo.priority == 1

        desc_bar = core.ReactionDescriptor(object(), list())
        desc_bar.__set_name__(Reactor, "bar")
        assert desc_bar.name == "bar"
        assert desc_bar.priority == 2

    @staticmethod
    def test_create_instance(mocker):
        env = core.Environment()
        reactor = env.create_reactor("reactor", Reactor)
        reaction_stub = mocker.Mock(spec=Callable)
        desc = core.ReactionDescriptor(reaction_stub, inspect.stack()[0])
        desc.__set_name__(Reactor, "foo")
        instance = desc.create_instance(reactor)
        reaction_stub.assert_called_once_with(reactor, mocker.ANY)
        assert instance.name == "foo"

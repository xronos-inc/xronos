# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# type: ignore

from typing import Callable

import pytest

import xronos._core as core


def test_decorator():
    reaction = object()
    descriptor = core.reaction(reaction)
    assert isinstance(descriptor, core.ReactionDescriptor)


class TestReactionDecorator:
    @staticmethod
    def test_init():
        reaction = object()
        desc = core.ReactionDescriptor(reaction, core.get_source_location())
        assert desc._ElementDescriptor__name is None

    @staticmethod
    def test_set_name():
        class Reactor(core.Reactor):
            pass

        desc_foo = core.ReactionDescriptor(object(), list())

        with pytest.raises(AssertionError):
            desc_foo._name

        desc_foo.__set_name__(Reactor, "foo")
        assert desc_foo._name == "foo"

        desc_bar = core.ReactionDescriptor(object(), list())
        desc_bar.__set_name__(Reactor, "bar")
        assert desc_bar._name == "bar"

    @staticmethod
    def test_create_instance(mocker):
        class Reactor(core.Reactor):
            pass

        env = core.Environment()
        reactor = env.create_reactor("reactor", Reactor)
        reaction_stub = mocker.Mock(spec=Callable)
        desc = core.ReactionDescriptor(reaction_stub, core.get_source_location())
        desc.__set_name__(Reactor, "foo")
        instance = desc._create_instance(reactor)
        assert not reaction_stub.called
        instance._assemble()
        reaction_stub.assert_called_once_with(reactor, mocker.ANY)
        assert instance.name == "foo"

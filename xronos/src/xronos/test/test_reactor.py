# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# type: ignore
# ruff: noqa: PLR2004

import typing

import pytest

import xronos


class BaseClass(xronos.Reactor):
    pass


class Class(BaseClass):
    pass


class ChildClass(Class):
    pass


class BaseClass2(xronos.Reactor):
    pass


class Mixin:
    pass


class MixinClass(Class, Mixin):
    pass


@pytest.fixture(params=[BaseClass, Class, ChildClass])
def class_(request: pytest.FixtureRequest) -> typing.Type[xronos.Reactor]:
    return typing.cast(typing.Type[xronos.Reactor], request.param)


class TestBaseReactor:
    @staticmethod
    def test_init(class_: typing.Type[xronos.Reactor]) -> None:
        assert hasattr(class_, "_Reactor__element_descriptors")
        assert isinstance(class_._Reactor__element_descriptors, list)
        assert len(class_._Reactor__element_descriptors) == 0

    @staticmethod
    def test_get_base() -> None:
        assert xronos.Reactor._Reactor__get_base_reactor() is None
        assert BaseClass._Reactor__get_base_reactor() is None
        assert Class._Reactor__get_base_reactor() is BaseClass
        assert ChildClass._Reactor__get_base_reactor() is Class
        assert MixinClass._Reactor__get_base_reactor() is Class

    @staticmethod
    def test_multiple_inheritance():
        with pytest.raises(NotImplementedError):

            class TwoBases(Class, BaseClass2):
                pass

    @staticmethod
    def test_register_descriptor():
        dummy_desc_1 = object()
        dummy_desc_2 = object()
        dummy_desc_3 = object()
        Class._register_element_descriptor(dummy_desc_1)
        Class._register_element_descriptor(dummy_desc_2)
        Class._register_element_descriptor(dummy_desc_3)
        assert len(Class._Reactor__element_descriptors) == 3
        assert len(BaseClass._Reactor__element_descriptors) == 0
        assert dummy_desc_1 is Class._Reactor__element_descriptors[0]
        assert dummy_desc_2 is Class._Reactor__element_descriptors[1]
        assert dummy_desc_3 is Class._Reactor__element_descriptors[2]

    @staticmethod
    def test_base_descriptors():
        assert len(BaseClass._Reactor__get_base_element_descriptors()) == 0
        assert len(Class._Reactor__get_base_element_descriptors()) == 0
        assert len(ChildClass._Reactor__get_base_element_descriptors()) == 3
        assert len(MixinClass._Reactor__get_base_element_descriptors()) == 3


def test_direct_initialization():
    # Assert that TypeError is raised when a reactor is instantiated directly
    with pytest.raises(TypeError):

        class Reactor(xronos.Reactor):
            pass

        Reactor()

    # But using create_reactor works
    env = xronos.Environment()
    env.create_reactor("reactor", Reactor)


def test_init_called():
    # Assert that TypeError is raised when user forgets calling `super.__init__()`
    with pytest.raises(TypeError):

        class Reactor(xronos.Reactor):
            def __init__(self) -> None:
                pass

        env = xronos.Environment()
        env.create_reactor("reactor", Reactor)

    # Assert that TypeError is raised when user forgets calling
    # `super.__init__()` or calls it only after accessing an element.
    with pytest.raises(TypeError):

        class Reactor(xronos.Reactor):
            t = xronos.PeriodicTimerDeclaration()

            def __init__(self) -> None:
                self.t.period = None
                super().__init__()

        env = xronos.Environment()
        env.create_reactor("reactor", Reactor)

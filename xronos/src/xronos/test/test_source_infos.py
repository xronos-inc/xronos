# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# Ignore missing source for xronos._runtime
# pyright: reportMissingModuleSource = false
# ruff: noqa: PLR2004

import sys
from typing import Callable

import xronos
from xronos._runtime import SourceInfo


def mock_handler():
    pass


class Foo(xronos.Reactor):
    timer = xronos.PeriodicTimerDeclaration()
    event_without_type = xronos.ProgrammableTimerDeclaration()  # pyright: ignore

    @xronos.reaction
    def foo_reaction(self, _: xronos.ReactionInterface) -> Callable[[], None]:
        return mock_handler


class Base(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()
    output = xronos.OutputPortDeclaration()  # pyright: ignore

    def __init__(self) -> None:
        super().__init__()
        self.foo = self.create_reactor("foo", Foo)

    @xronos.reaction
    def base_reaction(self, _: xronos.ReactionInterface) -> Callable[[], None]:
        return mock_handler


class Child(Base):
    event = xronos.ProgrammableTimerDeclaration[None]()

    @xronos.reaction
    def child_reaction(self, _: xronos.ReactionInterface) -> Callable[[], None]:
        return mock_handler


def test_source_locations():
    env = xronos.Environment()
    child = env.create_reactor("child", Child)
    env.execute()

    source_infos: dict[str, SourceInfo] = {
        ".".join(s.fqn): s  # pyright: ignore
        for s in env._Environment__source_locations  # pyright: ignore
    }

    for fqn, info in source_infos.items():
        if "startup" in fqn or "shutdown" in fqn:
            assert "_core.py" in info.file
        else:
            assert "test_source_infos.py" in info.file

    assert source_infos[child.fqn].class_name == Child.__name__
    assert source_infos[child.fqn].lineno == 51

    assert source_infos[child.event.fqn].lineno == 42
    assert source_infos[child.input_.fqn].lineno == 29
    assert source_infos[child.output.fqn].lineno == 30  # pyright: ignore
    assert source_infos[child.foo.fqn].lineno == 34
    assert source_infos[child.foo.timer.fqn].lineno == 20
    assert source_infos[child.foo.event_without_type.fqn].lineno == 21  # pyright: ignore

    # The reported line number for reactions differs slightly between Python versions
    if sys.version_info.minor > 10:
        assert source_infos[child.foo.foo_reaction.fqn].lineno == 23  # pyright: ignore
        assert source_infos[child.base_reaction.fqn].lineno == 36  # pyright: ignore
        assert source_infos[child.child_reaction.fqn].lineno == 44  # pyright: ignore
    else:
        assert source_infos[child.foo.foo_reaction.fqn].lineno == 24  # pyright: ignore
        assert source_infos[child.base_reaction.fqn].lineno == 37  # pyright: ignore
        assert source_infos[child.child_reaction.fqn].lineno == 45  # pyright: ignore

    assert source_infos[child.foo.fqn].class_name == Foo.__name__

# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause


import pytest

import xronos


class EmptyReactor(xronos.Reactor):
    pass


class ReactorWithOutput(xronos.Reactor):
    output = xronos.OutputPortDeclaration[None]()


def test_duplicate_reactor_names() -> None:
    env = xronos.Environment()
    test = env.create_reactor("test", EmptyReactor)
    with pytest.raises(xronos.DuplicateNameError):
        env.create_reactor("test", EmptyReactor)
    with pytest.raises(xronos.DuplicateNameError):
        test.create_reactor("startup", EmptyReactor)
    with pytest.raises(xronos.DuplicateNameError):
        foo = env.create_reactor("foo", ReactorWithOutput)
        foo.create_reactor("output", EmptyReactor)

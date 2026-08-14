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
    with pytest.raises(xronos.InvalidNameError):
        env.create_reactor("test", EmptyReactor)
    with pytest.raises(xronos.InvalidNameError):
        test.create_reactor("startup", EmptyReactor)
    foo = env.create_reactor("foo", ReactorWithOutput)
    with pytest.raises(xronos.InvalidNameError):
        foo.create_reactor("output", EmptyReactor)

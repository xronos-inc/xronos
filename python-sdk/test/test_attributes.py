# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause


import pytest

import xronos


class EmptyReactor(xronos.Reactor):
    pass


def test_add_attribute_rejects_duplicate_key() -> None:
    env = xronos.Environment()
    reactor = env.create_reactor("reactor", EmptyReactor)
    reactor.add_attribute("asil", "B")
    with pytest.raises(KeyError):
        reactor.add_attribute("asil", "D")


def test_add_attributes_rejects_duplicate_key() -> None:
    env = xronos.Environment()
    reactor = env.create_reactor("reactor", EmptyReactor)
    reactor.add_attribute("asil", "B")

    with pytest.raises(KeyError):
        reactor.add_attributes({"asil": "D", "revision": 3.0})

    # Attributes apply per key: "revision" was added despite the failure.
    with pytest.raises(KeyError):
        reactor.add_attribute("revision", 4.0)


def test_add_attributes_succeeds_without_duplicates() -> None:
    env = xronos.Environment()
    reactor = env.create_reactor("reactor", EmptyReactor)
    reactor.add_attributes({"asil": "B", "revision": 3.0})

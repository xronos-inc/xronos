# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import xronos


def run(env: xronos.Environment) -> None:
    env.execute()


def main(fast: bool = False) -> None:
    env = xronos.Environment(fast=fast)
    run(env)


def test_empty_program() -> None:
    env = xronos.Environment(fast=True)
    run(env)


if __name__ == "__main__":
    main()

// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

namespace xronos::sdk::test {

TEST(empty, EmptyEnvironment) {
  // Test succeeds if it executes without error
  TestEnvironment env{};
  env.execute();
}

TEST(empty, EmptyReactor) {
  // Test succeeds if it executes without error
  TestEnvironment env{};
  class EmptyReactor : public Reactor {
    using Reactor::Reactor;
    void assemble() final {}
  };

  EmptyReactor empty{"empty", env.context()};
  env.execute();
}

} // namespace xronos::sdk::test

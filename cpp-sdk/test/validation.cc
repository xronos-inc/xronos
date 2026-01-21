// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

namespace xronos::sdk::test {

TEST(validation, ThrowOnDuplicateReactorNames) {

  class MockReactor : public Reactor {
  public:
    using Reactor::Reactor;
    void assemble() final {}
  };

  TestEnvironment env{};

  auto foo = MockReactor("foo", env.context());
  auto bar = MockReactor("bar", env.context());
  EXPECT_THROW(MockReactor("foo", env.context()), DuplicateNameError);
  auto baz = MockReactor("baz", env.context());
  EXPECT_THROW(MockReactor("bar", env.context()), DuplicateNameError);
}

TEST(validation, ThrowOnDuplicateElementNames) {
  class BadReactor : public Reactor {
  public:
    using Reactor::Reactor;
    void assemble() final {}

  private:
    InputPort<void> input_{"input", context()};
    OutputPort<void> output_{"input", context()};
  };

  TestEnvironment env{};

  EXPECT_THROW(BadReactor("bad", env.context()), DuplicateNameError);

  class TestReactor : public Reactor {
  public:
    using Reactor::context;
    using Reactor::Reactor;

  private:
    class OnStartupReaction : public Reaction<TestReactor> {
      using Reaction<TestReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final {}
    };

    void assemble() final { add_reaction<OnStartupReaction>("startup"); }

    InputPort<void> input_{"input", context()};
    OutputPort<void> output_{"output", context()};
    PeriodicTimer timer_{"timer", context(), std::chrono::seconds{1}};
  };

  TestReactor test{"test", env.context()};
  EXPECT_THROW(PhysicalEvent<void>("timer", test.context()), DuplicateNameError);
  EXPECT_THROW(Metric("input", test.context(), "mock", "mock"), DuplicateNameError);
  EXPECT_THROW(Startup("timer", test.context()), DuplicateNameError);
  EXPECT_THROW(PeriodicTimer("shutdown", test.context(), std::chrono::seconds{1}), DuplicateNameError);

  EXPECT_THROW(env.execute(), DuplicateNameError);
}

} // namespace xronos::sdk::test

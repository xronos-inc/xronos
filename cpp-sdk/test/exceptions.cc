// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <stdexcept>
#include <string>

#include "xronos/sdk.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/reactor.hh"
#include "gtest/gtest.h"

namespace xronos::sdk::test {

TEST(exceptions, HandlerExceptionPropagatesAndShutdownRuns) {
  class ThrowingReactor : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto shutdown_reaction_executed() const noexcept -> bool { return shutdown_reaction_executed_; }

  private:
    bool shutdown_reaction_executed_{false};

    class OnStartupReaction : public Reaction<ThrowingReactor> {
      using Reaction<ThrowingReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final { throw std::runtime_error{"exception from startup handler"}; }
    };

    class OnShutdownReaction : public Reaction<ThrowingReactor> {
      using Reaction<ThrowingReactor>::Reaction;
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final { self().shutdown_reaction_executed_ = true; }
    };

    void assemble() final {
      add_reaction<OnStartupReaction>("on_startup");
      add_reaction<OnShutdownReaction>("on_shutdown");
    }
  };

  TestEnvironment env{};
  ThrowingReactor reactor{"reactor", env.context()};

  try {
    env.execute();
    FAIL() << "expected execute() to rethrow the handler exception";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "exception from startup handler");
  }

  EXPECT_TRUE(reactor.shutdown_reaction_executed());
}

TEST(exceptions, FirstExceptionWinsAndDroppedExceptionIsLogged) {
  class DoubleThrowingReactor : public Reactor {
  public:
    using Reactor::Reactor;

  private:
    class OnStartupReaction : public Reaction<DoubleThrowingReactor> {
      using Reaction<DoubleThrowingReactor>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final { throw std::runtime_error{"first exception"}; }
    };

    class OnShutdownReaction : public Reaction<DoubleThrowingReactor> {
      using Reaction<DoubleThrowingReactor>::Reaction;
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final { throw std::runtime_error{"second exception"}; }
    };

    void assemble() final {
      add_reaction<OnStartupReaction>("on_startup");
      add_reaction<OnShutdownReaction>("on_shutdown");
    }
  };

  TestEnvironment env{};
  DoubleThrowingReactor reactor{"reactor", env.context()};

  testing::internal::CaptureStderr();

  try {
    env.execute();
    FAIL() << "expected execute() to rethrow the handler exception";
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "first exception");
  }

  std::string captured_output = testing::internal::GetCapturedStderr();
  EXPECT_NE(captured_output.find("Dropping exception from reaction reactor.on_shutdown "
                                 "because an earlier exception is already recorded: second exception"),
            std::string::npos);
}

} // namespace xronos::sdk::test

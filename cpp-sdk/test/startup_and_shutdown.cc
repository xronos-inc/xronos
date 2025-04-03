// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

namespace xronos::sdk::test {

TEST(startup_and_shutdown, StartupTriggersOnce) {
  class StartupTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> bool { return reaction_executed_; }

  private:
    bool reaction_executed_{false};

    class OnStartupReaction : public Reaction<StartupTest> {
      using Reaction<StartupTest>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      void handler() final {
        EXPECT_FALSE(self().reaction_executed_);
        EXPECT_TRUE(startup_trigger_.is_present());
        self().reaction_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnStartupReaction>("on_startup"); }
  };

  TestEnvironment env{};
  StartupTest startup_test{"startup_test", env.context()};
  env.execute();

  EXPECT_TRUE(startup_test.reaction_executed());
}

TEST(startup_and_shutdown, ShutdownTriggersOnce) {
  class ShutdownTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> bool { return reaction_executed_; }

  private:
    bool reaction_executed_{false};

    class OnShutdownReaction : public Reaction<ShutdownTest> {
      using Reaction<ShutdownTest>::Reaction;
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final {
        EXPECT_FALSE(self().reaction_executed_);
        EXPECT_TRUE(shutdown_trigger_.is_present());
        self().reaction_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnShutdownReaction>("on_shutdown"); }
  };

  TestEnvironment env{};
  ShutdownTest shutdown_test{"shutdown_test", env.context()};
  env.execute();

  EXPECT_TRUE(shutdown_test.reaction_executed());
}

TEST(startup_and_shutdown, TestStartupAndShutdown) {
  class StartupAndShutdownTest : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto reaction_executed() const noexcept -> int { return reaction_executed_; }

  private:
    int reaction_executed_{0};
    TimePoint startup_time_{};

    class TestReaction : public Reaction<StartupAndShutdownTest> {
      using Reaction<StartupAndShutdownTest>::Reaction;
      Trigger<void> startup_trigger_{self().startup(), context()};
      Trigger<void> shutdown_trigger_{self().shutdown(), context()};
      void handler() final {
        EXPECT_TRUE(startup_trigger_.is_present() || shutdown_trigger_.is_present());
        EXPECT_FALSE(startup_trigger_.is_present() && shutdown_trigger_.is_present());
        if (startup_trigger_.is_present()) {
          self().startup_time_ = self().get_time();
          EXPECT_EQ(self().reaction_executed_, 0);
        }
        if (shutdown_trigger_.is_present()) {
          EXPECT_EQ(self().reaction_executed_, 1);
          EXPECT_EQ(self().startup_time_, self().get_time()); // no time passes between startup and shutdown
        }
        self().reaction_executed_++;
      }
    };

    void assemble() final { add_reaction<TestReaction>("on_startup_or_shutdown"); }
  };

  TestEnvironment env{};
  StartupAndShutdownTest shutdown_test{"startup_and_shutdown_test", env.context()};
  env.execute();

  EXPECT_EQ(shutdown_test.reaction_executed(), 2);
}

} // namespace xronos::sdk::test

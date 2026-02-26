// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <chrono>
#include <thread>

#include "xronos/sdk.hh"
#include "xronos/sdk/time.hh"

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

TEST(deadlines, TestDeadlineGetters) {
  class DeadlineGettersTest : public Reactor {
  private:
    using Reactor::Reactor;

    PeriodicTimer timer_{"timer", context(), 1s};

    TimePoint start_time_{};

    unsigned timer_count_{0};
    bool on_startup_executed_{false};
    bool on_shutdown_executed_{false};

    class OnStartup : public Reaction<DeadlineGettersTest> {
      using Reaction<DeadlineGettersTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      void handler() final {
        EXPECT_FALSE(self().on_startup_executed_);
        EXPECT_TRUE(deadline().has_value());
        EXPECT_EQ(deadline().value() - self().get_time(), 100ms);
        EXPECT_GT(remaining_slack(), 0s);
        EXPECT_TRUE(is_before_deadline());
        self().on_startup_executed_ = true;
      }
    };

    class OnShutdown : public Reaction<DeadlineGettersTest> {
      using Reaction<DeadlineGettersTest>::Reaction;
      Trigger<void> shutdown_trigger{self().shutdown(), context()};
      void handler() final {
        EXPECT_FALSE(self().on_shutdown_executed_);
        EXPECT_TRUE(deadline().has_value());
        EXPECT_EQ(deadline().value() - self().get_time(), 200ms);
        EXPECT_GT(remaining_slack(), 0s);
        EXPECT_TRUE(is_before_deadline());
        self().on_shutdown_executed_ = true;
      }
    };

    class OnTimer : public Reaction<DeadlineGettersTest> {
      using Reaction<DeadlineGettersTest>::Reaction;
      Trigger<void> timer_trigger{self().timer_, context()};
      void handler() final {
        EXPECT_TRUE(deadline().has_value());
        EXPECT_EQ(deadline().value() - self().get_time(), 50ms);
        EXPECT_GT(remaining_slack(), std::chrono::seconds{self().timer_count_});
        EXPECT_TRUE(is_before_deadline());
        self().timer_count_++;
      }
    };

    void assemble() final {
      add_reaction_with_deadline<OnStartup>("on_startup", 100ms);
      add_reaction_with_deadline<OnTimer>("on_timer", 50ms);
      add_reaction_with_deadline<OnShutdown>("on_shutdown", 200ms);
    }

  public:
    void check_post_conditions() const {
      EXPECT_TRUE(on_startup_executed_);
      EXPECT_TRUE(on_shutdown_executed_);
      EXPECT_EQ(timer_count_, 6);
    }
  };

  TestEnvironment env{5s};
  DeadlineGettersTest deadline_test{"deadline_test", env.context()};
  env.execute();
  deadline_test.check_post_conditions();
}

TEST(deadlines, TestDeadlineGettersNoDeadlines) {
  class DeadlineGettersTest : public Reactor {
  private:
    using Reactor::Reactor;

    PeriodicTimer timer_{"timer", context(), 1s};

    TimePoint start_time_{};

    unsigned timer_count_{0};
    bool on_startup_executed_{false};
    bool on_shutdown_executed_{false};

    class OnStartup : public Reaction<DeadlineGettersTest> {
      using Reaction<DeadlineGettersTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      void handler() final {
        EXPECT_FALSE(self().on_startup_executed_);
        EXPECT_FALSE(deadline().has_value());
        EXPECT_EQ(remaining_slack(), Duration::max());
        EXPECT_TRUE(is_before_deadline());
        self().on_startup_executed_ = true;
      }
    };

    class OnShutdown : public Reaction<DeadlineGettersTest> {
      using Reaction<DeadlineGettersTest>::Reaction;
      Trigger<void> shutdown_trigger{self().shutdown(), context()};
      void handler() final {
        EXPECT_FALSE(self().on_shutdown_executed_);
        EXPECT_FALSE(deadline().has_value());
        EXPECT_EQ(remaining_slack(), Duration::max());
        EXPECT_TRUE(is_before_deadline());
        self().on_shutdown_executed_ = true;
      }
    };

    class OnTimer : public Reaction<DeadlineGettersTest> {
      using Reaction<DeadlineGettersTest>::Reaction;
      Trigger<void> timer_trigger{self().timer_, context()};
      void handler() final {
        EXPECT_FALSE(deadline().has_value());
        EXPECT_EQ(remaining_slack(), Duration::max());
        EXPECT_TRUE(is_before_deadline());
        self().timer_count_++;
      }
    };

    void assemble() final {
      add_reaction<OnStartup>("on_startup");
      add_reaction<OnTimer>("on_timer");
      add_reaction<OnShutdown>("on_shutdown");
    }

  public:
    void check_post_conditions() const {
      EXPECT_TRUE(on_startup_executed_);
      EXPECT_TRUE(on_shutdown_executed_);
      EXPECT_EQ(timer_count_, 6);
    }
  };

  TestEnvironment env{5s};
  DeadlineGettersTest deadline_test{"deadline_test", env.context()};
  env.execute();
  deadline_test.check_post_conditions();
}

TEST(deadlines, TestViolatedDeadline) {

  class DeadlineViolatedTest : public Reactor {
  private:
    using Reactor::Reactor;

    PeriodicTimer timer_{"timer", context(), 1ms};

    TimePoint start_time_{};

    unsigned timer_count_{0};
    bool on_startup_executed_{false};
    bool on_shutdown_executed_{false};

    class OnStartup : public Reaction<DeadlineViolatedTest> {
      using Reaction<DeadlineViolatedTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      void handler() final {
        EXPECT_FALSE(self().on_startup_executed_);
        EXPECT_TRUE(deadline().has_value());
        EXPECT_EQ(deadline().value(), self().get_time());
        EXPECT_LT(remaining_slack(), 0s);
        EXPECT_FALSE(is_before_deadline());
        self().on_startup_executed_ = true;

        std::this_thread::sleep_for(10ms);
      }
    };

    class OnShutdown : public Reaction<DeadlineViolatedTest> {
      using Reaction<DeadlineViolatedTest>::Reaction;
      Trigger<void> shutdown_trigger{self().shutdown(), context()};
      void handler() final {
        EXPECT_FALSE(self().on_shutdown_executed_);
        EXPECT_TRUE(deadline().has_value());
        EXPECT_EQ(deadline().value() - self().get_time(), 200ms);
        EXPECT_GT(remaining_slack(), 0s);
        EXPECT_TRUE(is_before_deadline());
        self().on_shutdown_executed_ = true;
      }
    };

    class OnTimer : public Reaction<DeadlineViolatedTest> {
      using Reaction<DeadlineViolatedTest>::Reaction;
      Trigger<void> timer_trigger{self().timer_, context()};
      Duration previous_slack{-1s};

      void handler() final {
        EXPECT_TRUE(deadline().has_value());
        EXPECT_FALSE(is_before_deadline());

        auto slack = remaining_slack();
        EXPECT_EQ(deadline().value() - self().get_time(), 5ms);
        EXPECT_LT(slack, 0s);
        EXPECT_LT(previous_slack, slack);
        previous_slack = slack;

        self().timer_count_++;
      }
    };

    void assemble() final {
      add_reaction_with_deadline<OnStartup>("on_startup", 0s);
      add_reaction_with_deadline<OnTimer>("on_timer", 5ms);
      add_reaction_with_deadline<OnShutdown>("on_shutdown", 200ms);
    }

  public:
    void check_post_conditions() const {
      EXPECT_TRUE(on_startup_executed_);
      EXPECT_TRUE(on_shutdown_executed_);
      EXPECT_EQ(timer_count_, 6);
    }
  };

  TestEnvironment env{5ms};
  DeadlineViolatedTest deadline_test{"deadline_test", env.context()};
  env.execute();
  deadline_test.check_post_conditions();
}

} // namespace xronos::sdk::test

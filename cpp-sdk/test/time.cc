// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <thread>

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

TEST(time, TestTimGetters) {
  class GetTimeTest : public Reactor {
  private:
    using Reactor::Reactor;

    PeriodicTimer timer_{"timer", context(), 1s};

    TimePoint start_time_{};

    unsigned timer_count_{0};
    bool on_startup_executed_{false};
    bool on_shutdown_executed_{false};

    class OnStartup : public Reaction<GetTimeTest> {
      using Reaction<GetTimeTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      void handler() final {
        self().start_time_ = self().get_time();
        EXPECT_EQ(self().get_time_since_startup(), 0s);
        EXPECT_FALSE(self().on_startup_executed_);
        self().on_startup_executed_ = true;
      }
    };

    class OnShutdown : public Reaction<GetTimeTest> {
      using Reaction<GetTimeTest>::Reaction;
      Trigger<void> shutdown_trigger{self().shutdown(), context()};
      void handler() final {
        EXPECT_EQ(self().get_time_since_startup(), 5s);
        EXPECT_EQ(self().get_time_since_startup(), self().get_time() - self().start_time_);
        EXPECT_FALSE(self().on_shutdown_executed_);
        self().on_shutdown_executed_ = true;
      }
    };

    class OnTimer : public Reaction<GetTimeTest> {
      using Reaction<GetTimeTest>::Reaction;
      Trigger<void> timer_trigger{self().timer_, context()};
      void handler() final {
        EXPECT_EQ(self().get_time_since_startup(), self().timer_count_ * 1s);
        EXPECT_EQ(self().get_time(), self().start_time_ + self().timer_count_ * 1s);
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
  GetTimeTest timer_test{"timer_test", env.context()};
  env.execute();
  timer_test.check_post_conditions();
}

TEST(time, TimeDoesNotPassInReaction) {
  class TimeDoesNotPassTest : public Reactor {
  private:
    using Reactor::Reactor;

    bool on_startup_executed_{false};

    class OnStartup : public Reaction<TimeDoesNotPassTest> {
      using Reaction<TimeDoesNotPassTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      void handler() final {
        auto start_time_ = self().get_time();
        EXPECT_EQ(self().get_time_since_startup(), 0s);
        EXPECT_EQ(self().get_time() - start_time_, 0s);
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(self().get_time_since_startup(), 0s);
        EXPECT_EQ(self().get_time() - start_time_, 0s);

        EXPECT_FALSE(self().on_startup_executed_);
        self().on_startup_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnStartup>("on_startup"); }

  public:
    void check_post_conditions() const { EXPECT_TRUE(on_startup_executed_); }
  };

  TestEnvironment env{5s};
  TimeDoesNotPassTest test{"test", env.context()};
  env.execute();
  test.check_post_conditions();
}

TEST(time, TestGetLag) {
  class GetLagTest : public Reactor {
    using Reactor::Reactor;

    bool on_startup_executed_{false};

    class OnStartup : public Reaction<GetLagTest> {
      using Reaction<GetLagTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      void handler() final {
        EXPECT_GT(self().get_lag(), 0s);
        auto lag1 = self().get_lag();
        auto lag2 = self().get_lag();
        EXPECT_GE(lag2, lag1);
        EXPECT_LT(lag2 - lag1, 1ms);
        std::this_thread::sleep_for(100ms);
        auto lag3 = self().get_lag();
        EXPECT_GT(lag3, lag2);
        EXPECT_GE(lag3 - lag2, 100ms);
        EXPECT_LT(lag3 - lag2, 200ms);

        EXPECT_FALSE(self().on_startup_executed_);
        self().on_startup_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnStartup>("on_startup"); }

  public:
    void check_post_conditions() const { EXPECT_TRUE(on_startup_executed_); }
  };

  TestEnvironment env{};
  GetLagTest test{"test", env.context()};
  env.execute();
  test.check_post_conditions();
}

} // namespace xronos::sdk::test

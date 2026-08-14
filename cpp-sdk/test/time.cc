// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <sstream>
#include <string>
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
        self().start_time_ = current_time();
        EXPECT_EQ(elapsed_time(), 0s);
        EXPECT_FALSE(self().on_startup_executed_);
        self().on_startup_executed_ = true;
      }
    };

    class OnShutdown : public Reaction<GetTimeTest> {
      using Reaction<GetTimeTest>::Reaction;
      Trigger<void> shutdown_trigger{self().shutdown(), context()};
      void handler() final {
        EXPECT_EQ(elapsed_time(), 5s);
        EXPECT_EQ(elapsed_time(), current_time() - self().start_time_);
        EXPECT_FALSE(self().on_shutdown_executed_);
        self().on_shutdown_executed_ = true;
      }
    };

    class OnTimer : public Reaction<GetTimeTest> {
      using Reaction<GetTimeTest>::Reaction;
      Trigger<void> timer_trigger{self().timer_, context()};
      void handler() final {
        EXPECT_EQ(elapsed_time(), self().timer_count_ * 1s);
        EXPECT_EQ(current_time(), self().start_time_ + self().timer_count_ * 1s);
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
        auto start_time_ = current_time();
        EXPECT_EQ(elapsed_time(), 0s);
        EXPECT_EQ(current_time() - start_time_, 0s);
        std::this_thread::sleep_for(100ms);
        EXPECT_EQ(elapsed_time(), 0s);
        EXPECT_EQ(current_time() - start_time_, 0s);

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
        EXPECT_GT(lag(), 0s);
        auto lag1 = lag();
        auto lag2 = lag();
        EXPECT_GE(lag2, lag1);
        EXPECT_LT(lag2 - lag1, 1ms);
        std::this_thread::sleep_for(100ms);
        auto lag3 = lag();
        EXPECT_GT(lag3, lag2);
        EXPECT_GE(lag3 - lag2, 100ms);
        EXPECT_LT(lag3 - lag2, 300ms);

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

// The reaction-scoped timing API remains callable during a reaction's
// construction / member initialization, where the runtime is not yet running.
// There it must return the defined placeholders (epoch / zero).
TEST(time, InitTimeDefaults) {
  class InitDefaultsTest : public Reactor {
    using Reactor::Reactor;

    bool on_startup_executed_{false};

    class OnStartup : public Reaction<InitDefaultsTest> {
      using Reaction<InitDefaultsTest>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};

      // Captured while the reaction is being constructed (during assembly),
      // before the runtime starts. These must be the defined placeholders.
      TimePoint construction_time_{current_time()};
      Duration construction_lag_{lag()};
      Duration construction_time_since_startup_{elapsed_time()};

      void handler() final {
        EXPECT_EQ(construction_time_, TimePoint{});
        EXPECT_EQ(construction_lag_, 0s);
        EXPECT_EQ(construction_time_since_startup_, 0s);

        EXPECT_FALSE(self().on_startup_executed_);
        self().on_startup_executed_ = true;
      }
    };

    void assemble() final { add_reaction<OnStartup>("on_startup"); }

  public:
    void check_post_conditions() const { EXPECT_TRUE(on_startup_executed_); }
  };

  TestEnvironment env{};
  InitDefaultsTest test{"test", env.context()};
  env.execute();
  test.check_post_conditions();
}

TEST(time, StreamTimePointFormatsDate) {
  std::ostringstream out;
  out << TimePoint{}; // epoch
  // Formatted as "YYYY-MM-DD HH:MM:SS.nnnnnnnnn".
  EXPECT_NE(out.str().find('-'), std::string::npos);
  EXPECT_NE(out.str().find('.'), std::string::npos);
}

// Streaming the extreme TimePoint values must not crash. They stay within
// std::localtime's range on a 64-bit time_t (years 1677-2262), so they format
// normally; this guards the formatting/arithmetic and the localtime null-check
// against regressions.
TEST(time, StreamTimePointExtremesDoNotCrash) {
  std::ostringstream out;
  out << TimePoint::max();
  out << TimePoint::min();
  EXPECT_FALSE(out.str().empty());
}

} // namespace xronos::sdk::test

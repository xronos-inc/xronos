// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

class ProgrammableTimerTest : public ::testing::TestWithParam<Duration> {
protected:
  class TestProgrammableTimerReactor : public Reactor {
    using Reactor::Reactor;

    ProgrammableTimer<void> programmable_timer_void_{"programmable_timer_void_", context()};
    ProgrammableTimer<int> programmable_timer_int_{"programmable_timer_int_", context()};

    PeriodicTimer reference_{"reference", context(), GetParam()};

    int expected_{0};

    class OnStartup : public Reaction<TestProgrammableTimerReactor> {
      using Reaction<TestProgrammableTimerReactor>::Reaction;
      Trigger<void> startup_trigger{self().startup(), context()};
      ProgrammableTimerEffect<void> void_effect{self().programmable_timer_void_, context()};
      ProgrammableTimerEffect<int> int_effect{self().programmable_timer_int_, context()};
      void handler() final {
        // schedule without delay
        void_effect.schedule();
        int_effect.schedule(0);
      }
    };

    class MatchWithTimer : public Reaction<TestProgrammableTimerReactor> {
      using Reaction<TestProgrammableTimerReactor>::Reaction;
      Trigger<void> reference_trigger{self().reference_, context()};
      Source<void> void_source{self().programmable_timer_void_, context()};
      Source<int> int_source{self().programmable_timer_int_, context()};
      void handler() final {
        EXPECT_TRUE(reference_trigger.is_present());
        if (self().get_time_since_startup() == 0s) {
          // The first iteration does not align with the first timer triggering
          EXPECT_FALSE(void_source.is_present());
          EXPECT_FALSE(int_source.is_present());
          EXPECT_EQ(int_source.get(), nullptr);
        } else {
          EXPECT_TRUE(void_source.is_present());
          EXPECT_TRUE(int_source.is_present());
          ASSERT_NE(int_source.get(), nullptr);
          EXPECT_EQ(*int_source.get(), self().expected_);
        }
      }
    };

    class TestPeriodicTimers : public Reaction<TestProgrammableTimerReactor> {
      using Reaction<TestProgrammableTimerReactor>::Reaction;
      Trigger<void> void_trigger{self().programmable_timer_void_, context()};
      Trigger<int> int_trigger{self().programmable_timer_int_, context()};
      Source<void> reference_source{self().reference_, context()};
      ProgrammableTimerEffect<void> void_effect{self().programmable_timer_void_, context()};
      ProgrammableTimerEffect<int> int_effect{self().programmable_timer_int_, context()};
      void handler() final {
        EXPECT_TRUE(void_trigger.is_present());
        EXPECT_TRUE(int_trigger.is_present());
        if (self().get_time_since_startup() == 0s) {
          // The first iteration does not align with the first timer triggering
          EXPECT_FALSE(reference_source.is_present());
        } else {
          EXPECT_TRUE(reference_source.is_present());
        }
        ASSERT_NE(int_trigger.get(), nullptr);
        EXPECT_EQ(*int_trigger.get(), self().expected_);

        self().expected_++;

        void_effect.schedule(GetParam());
        int_effect.schedule(self().expected_, GetParam());
      }
    };
    void assemble() final {
      add_reaction<OnStartup>("on_startup");
      add_reaction<MatchWithTimer>("match_with_timer");
      add_reaction<TestPeriodicTimers>("test");
    }
  };
};

TEST_P(ProgrammableTimerTest, run) {
  TestEnvironment env{5s};
  TestProgrammableTimerReactor test{"test", env.context()};
  env.execute();
}

INSTANTIATE_TEST_SUITE_P(programmable_timers, ProgrammableTimerTest, ::testing::Values(1s, 200ms, 3s));

} // namespace xronos::sdk::test

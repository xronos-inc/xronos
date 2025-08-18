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
      Trigger<void> void_trigger{self().programmable_timer_void_, context()};
      Trigger<int> int_trigger{self().programmable_timer_int_, context()};
      void handler() final {
        if (self().get_time_since_startup() == 0s && reference_trigger.is_present()) {
          // The first iteration does not align with the first timer triggering
          EXPECT_FALSE(void_trigger.is_present());
          EXPECT_FALSE(int_trigger.is_present());
          EXPECT_EQ(int_trigger.get(), nullptr);
        } else {
          if (self().get_time_since_startup() > 0s) {
            EXPECT_TRUE(reference_trigger.is_present());
          }
          EXPECT_TRUE(void_trigger.is_present());
          EXPECT_TRUE(int_trigger.is_present());
          ASSERT_NE(int_trigger.get(), nullptr);
          EXPECT_EQ(*int_trigger.get(), self().expected_);
        }
      }
    };

    class TestPeriodicTimers : public Reaction<TestProgrammableTimerReactor> {
      using Reaction<TestProgrammableTimerReactor>::Reaction;
      Trigger<void> void_trigger{self().programmable_timer_void_, context()};
      Trigger<int> int_trigger{self().programmable_timer_int_, context()};
      Trigger<void> reference_trigger{self().reference_, context()};
      ProgrammableTimerEffect<void> void_effect{self().programmable_timer_void_, context()};
      ProgrammableTimerEffect<int> int_effect{self().programmable_timer_int_, context()};
      void handler() final {
        if (self().get_time_since_startup() == 0s && reference_trigger.is_present()) {
          // The first iteration does not align with the first timer triggering
          EXPECT_FALSE(void_trigger.is_present());
          EXPECT_FALSE(int_trigger.is_present());
          EXPECT_EQ(int_trigger.get(), nullptr);
        } else {
          if (self().get_time_since_startup() > 0s) {
            EXPECT_TRUE(reference_trigger.is_present());
          }
          EXPECT_TRUE(void_trigger.is_present());
          EXPECT_TRUE(int_trigger.is_present());
          ASSERT_NE(int_trigger.get(), nullptr);
          EXPECT_EQ(*int_trigger.get(), self().expected_);
        }

        if (int_trigger.is_present() && void_trigger.is_present()) {
          self().expected_++;

          void_effect.schedule(GetParam());
          int_effect.schedule(self().expected_, GetParam());
        }
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

class ProgrammableTimerAsDelayTest : public ::testing::TestWithParam<Duration> {
protected:
  class Delay : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto& input() noexcept { return input_; }
    [[nodiscard]] auto& output_void() noexcept { return output_void_; }
    [[nodiscard]] auto& output_int() noexcept { return output_int_; }

  private:
    ProgrammableTimer<void> programmable_timer_void_{"programmable_timer_void_", context()};
    ProgrammableTimer<int> programmable_timer_int_{"programmable_timer_int_", context()};

    InputPort<void> input_{"input", context()};

    OutputPort<void> output_void_{"output_void", context()};
    OutputPort<int> output_int_{"output_int", context()};

    int expected_{0};

    class OnInput : public Reaction<Delay> {
      using Reaction<Delay>::Reaction;
      Trigger<void> input_trigger{self().input_, context()};
      ProgrammableTimerEffect<void> void_effect{self().programmable_timer_void_, context()};
      ProgrammableTimerEffect<int> int_effect{self().programmable_timer_int_, context()};
      void handler() final {
        EXPECT_TRUE(input_trigger.is_present());
        void_effect.schedule(GetParam());
        int_effect.schedule(++self().expected_, GetParam());
      }
    };

    class OnVoidTimer : public Reaction<Delay> {
      using Reaction<Delay>::Reaction;
      Trigger<void> trigger{self().programmable_timer_void_, context()};
      PortEffect<void> port_effect{self().output_void_, context()};
      void handler() final {
        EXPECT_TRUE(trigger.is_present());
        EXPECT_FALSE(port_effect.is_present());
        port_effect.set();
      }
    };

    class OnIntTimer : public Reaction<Delay> {
      using Reaction<Delay>::Reaction;
      Trigger<int> trigger{self().programmable_timer_int_, context()};
      PortEffect<int> port_effect{self().output_int_, context()};
      void handler() final {
        EXPECT_TRUE(trigger.is_present());
        EXPECT_FALSE(port_effect.is_present());
        port_effect.set(trigger.get());
      }
    };

    void assemble() final {
      add_reaction<OnInput>("on_input");
      add_reaction<OnVoidTimer>("on_void_timer");
      add_reaction<OnIntTimer>("on_int_timer");
    }
  };

  class SimpleTimerSource : public Reactor {
  public:
    using Reactor::Reactor;

    [[nodiscard]] auto& output() noexcept { return output_; }

  private:
    PeriodicTimer timer_{"timer", context(), GetParam()};
    OutputPort<void> output_{"output", context()};

    class OnTimer : public Reaction<SimpleTimerSource> {
      using Reaction<SimpleTimerSource>::Reaction;
      Trigger<void> timer_trigger{self().timer_, context()};
      PortEffect<void> output_effect{self().output_, context()};
      void handler() final { output_effect.set(); }
    };

    void assemble() final { add_reaction<OnTimer>("on_timer"); }
  };

  class Receiver : public Reactor {
  public:
    using Reactor::Reactor;
    [[nodiscard]] auto& input_void() noexcept { return input_void_; }
    [[nodiscard]] auto& input_int() noexcept { return input_int_; }

  private:
    PeriodicTimer reference_{"reference", context(), GetParam()};
    InputPort<void> input_void_{"input_void", context()};
    InputPort<int> input_int_{"input_int", context()};

    unsigned expected_{1};

    class MatchWithTimer : public Reaction<Receiver> {
      using Reaction<Receiver>::Reaction;
      Trigger<void> reference_trigger{self().reference_, context()};
      Trigger<void> void_trigger{self().input_void_, context()};
      Trigger<int> int_trigger{self().input_int_, context()};
      void handler() final {
        if (self().get_time_since_startup() == 0s) {
          // The first iteration does not align with the first timer triggering
          EXPECT_TRUE(reference_trigger.is_present());
          EXPECT_FALSE(void_trigger.is_present());
          EXPECT_FALSE(int_trigger.is_present());
          EXPECT_EQ(int_trigger.get(), nullptr);
        } else {
          EXPECT_TRUE(reference_trigger.is_present());
          EXPECT_TRUE(void_trigger.is_present());
          EXPECT_TRUE(int_trigger.is_present());
          ASSERT_NE(int_trigger.get(), nullptr);
          EXPECT_EQ(*int_trigger.get(), self().expected_++);
        }
      }
    };

    void assemble() final { add_reaction<MatchWithTimer>("match_with_timer"); }
  };
};

TEST_P(ProgrammableTimerAsDelayTest, run) {
  TestEnvironment env{5s};
  SimpleTimerSource source{"source", env.context()};
  Delay delay{"delay", env.context()};
  Receiver receiver{"receiver", env.context()};
  env.connect(source.output(), delay.input());
  env.connect(delay.output_void(), receiver.input_void());
  env.connect(delay.output_int(), receiver.input_int());

  env.execute();
}

INSTANTIATE_TEST_SUITE_P(delay_test, ProgrammableTimerAsDelayTest, ::testing::Values(1s, 200ms, 3s));

} // namespace xronos::sdk::test

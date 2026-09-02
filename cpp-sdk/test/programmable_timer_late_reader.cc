// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// Tests that every reaction triggered by a ProgrammableTimer reads the
// scheduled value, even when an earlier reaction at the same tag declares an
// effect on that timer. The timer's value must stay readable until the last
// triggered reaction at the tag has run, whether or not the effect-holding
// reaction executes.

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

namespace {

constexpr Duration delay = 1ms;
constexpr int num_values = 3;

// A writer reaction advances the timer; a later reader consumes the value.
// The reader must see the value even though the writer, which finishes
// first, holds an effect on the timer.
class WriterThenReader : public Reactor {
public:
  using Reactor::Reactor;
  void check_post_conditions() const {
    EXPECT_EQ(reader_count_, num_values);
    EXPECT_EQ(reader_sum_, num_values * (num_values + 1) / 2);
  }

private:
  ProgrammableTimer<int> timer_{"timer", context()};
  int reader_count_{0};
  int reader_sum_{0};

  class OnStartup : public Reaction<WriterThenReader> {
    using Reaction<WriterThenReader>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    ProgrammableTimerEffect<int> timer_effect{self().timer_, context()};
    void handler() final { timer_effect.schedule(1, delay); }
  };

  class OnTimer : public Reaction<WriterThenReader> {
    using Reaction<WriterThenReader>::Reaction;
    Trigger<int> timer_trigger{self().timer_, context()};
    ProgrammableTimerEffect<int> timer_effect{self().timer_, context()};
    void handler() final {
      const auto value = timer_trigger.get();
      ASSERT_TRUE(static_cast<bool>(value));
      if (*value < num_values) {
        timer_effect.schedule(*value + 1, delay);
      }
    }
  };

  class OnTimerLate : public Reaction<WriterThenReader> {
    using Reaction<WriterThenReader>::Reaction;
    Trigger<int> timer_trigger{self().timer_, context()};
    void handler() final {
      const auto value = timer_trigger.get();
      ASSERT_TRUE(static_cast<bool>(value));
      self().reader_sum_ += *value;
      self().reader_count_++;
    }
  };

  void assemble() final {
    add_reaction<OnStartup>("on_startup");
    add_reaction<OnTimer>("on_timer");
    add_reaction<OnTimerLate>("on_timer_late");
  }
};

// The writer declares an effect on the timer but is not triggered at the
// timer's tag, so it never executes there. Both readers must still see the
// value.
class ReadersAroundIdleWriter : public Reactor {
public:
  using Reactor::Reactor;
  void check_post_conditions() const {
    EXPECT_EQ(early_count_, 1);
    EXPECT_EQ(early_value_, 42);
    EXPECT_EQ(late_count_, 1);
    EXPECT_EQ(late_value_, 42);
  }

private:
  ProgrammableTimer<int> timer_{"timer", context()};
  int early_count_{0};
  int early_value_{0};
  int late_count_{0};
  int late_value_{0};

  class OnTimerEarly : public Reaction<ReadersAroundIdleWriter> {
    using Reaction<ReadersAroundIdleWriter>::Reaction;
    Trigger<int> timer_trigger{self().timer_, context()};
    void handler() final {
      const auto value = timer_trigger.get();
      ASSERT_TRUE(static_cast<bool>(value));
      self().early_value_ = *value;
      self().early_count_++;
    }
  };

  class OnStartup : public Reaction<ReadersAroundIdleWriter> {
    using Reaction<ReadersAroundIdleWriter>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    ProgrammableTimerEffect<int> timer_effect{self().timer_, context()};
    void handler() final { timer_effect.schedule(42, delay); }
  };

  class OnTimerLate : public Reaction<ReadersAroundIdleWriter> {
    using Reaction<ReadersAroundIdleWriter>::Reaction;
    Trigger<int> timer_trigger{self().timer_, context()};
    void handler() final {
      const auto value = timer_trigger.get();
      ASSERT_TRUE(static_cast<bool>(value));
      self().late_value_ = *value;
      self().late_count_++;
    }
  };

  void assemble() final {
    add_reaction<OnTimerEarly>("on_timer_early");
    add_reaction<OnStartup>("on_startup");
    add_reaction<OnTimerLate>("on_timer_late");
  }
};

TEST(programmable_timer_late_reader, ReaderAfterWriterSeesValue) {
  TestEnvironment env{(num_values + 2) * delay};
  WriterThenReader reactor{"writer_then_reader", env.context()};
  env.execute();
  reactor.check_post_conditions();
}

TEST(programmable_timer_late_reader, ReaderAfterIdleWriterSeesValue) {
  TestEnvironment env{3 * delay};
  ReadersAroundIdleWriter reactor{"readers_around_idle_writer", env.context()};
  env.execute();
  reactor.check_post_conditions();
}

} // namespace

} // namespace xronos::sdk::test

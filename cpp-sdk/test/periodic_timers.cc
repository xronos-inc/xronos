// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "xronos/sdk.hh"

#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

constexpr Duration timeout = 5s;

class PeriodicTimerTestReactor : public Reactor {
public:
  PeriodicTimerTestReactor(std::string_view name, Context parent_context, Duration period, Duration offset)
      : Reactor{name, parent_context}
      , timer_{"timer", context(), period, offset}
      , offset_timer_{"offset_timer_", context(), period, offset + 10ms}
      , period_{period}
      , offset_{offset} {}

  void check_post_conditions() {
    unsigned expected = 0;
    if (offset_ <= timeout) {
      expected = (timeout - offset_) / period_ + 1;
    }
    EXPECT_EQ(counter_, expected);
    EXPECT_EQ(offset_, timer_.offset());
    EXPECT_EQ(period_, timer_.period());
  }

private:
  PeriodicTimer timer_;
  PeriodicTimer offset_timer_;
  Duration period_;
  Duration offset_;

  unsigned counter_{0};

  class OnTimer : public Reaction<PeriodicTimerTestReactor> {
    using Reaction<PeriodicTimerTestReactor>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    void handler() final {
      EXPECT_TRUE(timer_trigger.is_present());
      EXPECT_EQ(self().get_time_since_startup(), self().offset_ + self().period_ * self().counter_);
      self().counter_++;
    }
  };

  class CheckIsPresent : public Reaction<PeriodicTimerTestReactor> {
    using Reaction<PeriodicTimerTestReactor>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    Trigger<void> offset_timer_trigger{self().offset_timer_, context()};
    void handler() final {
      EXPECT_TRUE(timer_trigger.is_present() || offset_timer_trigger.is_present());
      EXPECT_FALSE(timer_trigger.is_present() && offset_timer_trigger.is_present());
    }
  };

  void assemble() final {
    add_reaction<OnTimer>("on_timer");
    add_reaction<CheckIsPresent>("check_is_present");
  }
};

class PeriodicTimerTest : public ::testing::TestWithParam<std::pair<Duration, Duration>> {};

TEST_P(PeriodicTimerTest, run) {
  TestEnvironment env{timeout};
  PeriodicTimerTestReactor test{"test", env.context(), GetParam().first, GetParam().second};
  env.execute();
  test.check_post_conditions();
}

INSTANTIATE_TEST_SUITE_P(periodic_timers, PeriodicTimerTest,
                         ::testing::Values(std::make_pair(1s, 0s), std::make_pair(1s, 500ms), std::make_pair(1s, 1s),
                                           std::make_pair(100ms, 0s), std::make_pair(100ms, 500ms),
                                           std::make_pair(100ms, 1s), std::make_pair(1s, 10s), std::make_pair(10s, 0s),
                                           std::make_pair(10s, 10s)));

class InvalidPeriodicTimerTest : public ::testing::TestWithParam<std::pair<Duration, Duration>> {};

TEST_P(InvalidPeriodicTimerTest, run) {
  TestEnvironment env{timeout};
  PeriodicTimerTestReactor test{"test", env.context(), GetParam().first, GetParam().second};
  EXPECT_THROW(env.execute(), std::runtime_error);
}

INSTANTIATE_TEST_SUITE_P(periodic_timers, InvalidPeriodicTimerTest,
                         ::testing::Values(std::make_pair(0s, 0s), std::make_pair(-1s, 0s), std::make_pair(1s, -1s)));

} // namespace xronos::sdk::test

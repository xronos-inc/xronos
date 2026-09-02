// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <string_view>

#include "xronos/sdk.hh"
#include "gtest/gtest.h"

using namespace std::literals::chrono_literals;

namespace xronos::sdk::test {

// A reactor whose single reaction is triggered by its single timer and by
// shutdown. Nothing else constrains when that reaction may run, so a runtime
// that bounds execution only through inter-reaction synchronization has
// nothing to hold it back. The timer is merely the vehicle here; these tests
// are about the timeout bounding execution and placing the shutdown tag.
//
// Shutdown coordination engages only in programs that declare at least one
// shutdown effect. With `declare_idle_shutdown_effect` set, a second reaction
// declares a shutdown effect that it never triggers, so the tests cover the
// timeout with and without the coordination engaged. The effect lives on its
// own reaction because a reaction triggered by shutdown may not declare
// effects. The effect-free tests must stay effect-free: the coordination
// gate holds every reaction back until the shutdown tag resolves, so it
// would by itself stop a tick past the timeout and mask a broken timeout
// bound.
class LoneTimerReactor : public Reactor {
public:
  LoneTimerReactor(std::string_view name, Context parent_context, Duration period, Duration offset,
                   bool declare_idle_shutdown_effect = false)
      : Reactor{name, parent_context}
      , timer_{"timer", context(), period, offset}
      , declare_idle_shutdown_effect_{declare_idle_shutdown_effect} {}

  [[nodiscard]] auto tick_count() const noexcept -> unsigned { return tick_count_; }
  [[nodiscard]] auto last_tick_time() const noexcept -> Duration { return last_tick_time_; }
  [[nodiscard]] auto shutdown_count() const noexcept -> unsigned { return shutdown_count_; }
  [[nodiscard]] auto shutdown_time() const noexcept -> Duration { return shutdown_time_; }

private:
  PeriodicTimer timer_;
  bool declare_idle_shutdown_effect_;
  unsigned tick_count_{0};
  Duration last_tick_time_{Duration::zero()};
  unsigned shutdown_count_{0};
  Duration shutdown_time_{Duration::zero()};

  void record(bool timer_present, bool shutdown_present, Duration elapsed) {
    // The timeout shutdown gets its own tag, so exactly one trigger is
    // present, and nothing runs after shutdown.
    EXPECT_NE(timer_present, shutdown_present);
    EXPECT_EQ(shutdown_count_, 0U);
    if (timer_present) {
      tick_count_++;
      last_tick_time_ = elapsed;
    } else {
      shutdown_count_++;
      shutdown_time_ = elapsed;
    }
  }

  class OnTimerOrShutdown : public Reaction<LoneTimerReactor> {
    using Reaction<LoneTimerReactor>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    Trigger<void> shutdown_trigger{self().shutdown(), context()};
    void handler() final { self().record(timer_trigger.is_present(), shutdown_trigger.is_present(), elapsed_time()); }
  };

  class IdleShutdownEffectReaction : public Reaction<LoneTimerReactor> {
    using Reaction<LoneTimerReactor>::Reaction;
    Trigger<void> timer_trigger{self().timer_, context()};
    ShutdownEffect shutdown_effect_{self().shutdown(), context()};
    void handler() final {}
  };

  void assemble() final {
    add_reaction<OnTimerOrShutdown>("on_timer_or_shutdown");
    if (declare_idle_shutdown_effect_) {
      add_reaction<IdleShutdownEffectReaction>("idle_shutdown_effect");
    }
  }
};

// A timer whose first tag lies an hour past the timeout must never fire,
// however little else the program has to do. With nothing to do, shutdown
// lands right after the start tag instead of riding out the timeout.
TEST(Timeout, ATimerBeyondTheTimeoutNeverFires) {
  TestEnvironment env{50ms};
  LoneTimerReactor test{"test", env.context(), 1h, 1h};
  env.execute();
  EXPECT_EQ(test.tick_count(), 0U);
  EXPECT_EQ(test.shutdown_count(), 1U);
  EXPECT_EQ(test.shutdown_time(), 0s);
}

// An event at exactly the timeout still fires. The timer's second tag lies
// past the timeout, so it fires exactly once, and shutdown follows one
// microstep later at the same timestamp.
TEST(Timeout, ATimerAtTheTimeoutFires) {
  TestEnvironment env{50ms};
  LoneTimerReactor test{"test", env.context(), 50ms, 50ms};
  env.execute();
  EXPECT_EQ(test.tick_count(), 1U);
  EXPECT_EQ(test.last_tick_time(), 50ms);
  EXPECT_EQ(test.shutdown_count(), 1U);
  EXPECT_EQ(test.shutdown_time(), 50ms);
}

// One millisecond past the timeout is past it. As above, shutdown lands
// right after the start tag.
TEST(Timeout, ATimerJustPastTheTimeoutNeverFires) {
  TestEnvironment env{50ms};
  LoneTimerReactor test{"test", env.context(), 51ms, 51ms};
  env.execute();
  EXPECT_EQ(test.tick_count(), 0U);
  EXPECT_EQ(test.shutdown_count(), 1U);
  EXPECT_EQ(test.shutdown_time(), 0s);
}

// Declaring a shutdown effect engages shutdown coordination. The timeout
// must bound execution all the same, and the tags must match the effect-free
// case, even though no shutdown request ever comes.
TEST(Timeout, ATimerAtTheTimeoutFiresWithAnIdleShutdownEffect) {
  TestEnvironment env{50ms};
  LoneTimerReactor test{"test", env.context(), 50ms, 50ms, true};
  env.execute();
  EXPECT_EQ(test.tick_count(), 1U);
  EXPECT_EQ(test.last_tick_time(), 50ms);
  EXPECT_EQ(test.shutdown_count(), 1U);
  EXPECT_EQ(test.shutdown_time(), 50ms);
}

// As above with nothing to do at all: the program shuts down right after the
// start tag rather than stalling on a shutdown request that never comes.
TEST(Timeout, ATimerBeyondTheTimeoutNeverFiresWithAnIdleShutdownEffect) {
  TestEnvironment env{50ms};
  LoneTimerReactor test{"test", env.context(), 1h, 1h, true};
  env.execute();
  EXPECT_EQ(test.tick_count(), 0U);
  EXPECT_EQ(test.shutdown_count(), 1U);
  EXPECT_EQ(test.shutdown_time(), 0s);
}

} // namespace xronos::sdk::test

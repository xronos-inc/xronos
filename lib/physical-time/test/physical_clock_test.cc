// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// Pins the two clocks. SystemClock::wait_until returns true once the time
// point passed and false as soon as it is interrupted. FakeClock moves only
// through set_now and advance, and a waiter blocks until one of them carries
// the time to its time point.

#include <atomic>
#include <chrono>
#include <stop_token>
#include <thread>

#include "xronos/core/time.hh"
#include "xronos/physical_time/interrupt.hh"
#include "xronos/physical_time/physical_clock.hh"
#include "xronos/physical_time/testing/fake_clock.hh"
#include "gtest/gtest.h"

namespace {

using namespace std::chrono_literals;
using xronos::core::TimePoint;
using xronos::physical_time::FakeClock;
using xronos::physical_time::InterruptSource;
using xronos::physical_time::InterruptToken;
using xronos::physical_time::SystemClock;

auto time_at(std::chrono::seconds seconds) -> TimePoint { return TimePoint{} + seconds; }

TEST(SystemClockTest, WaitUntilReturnsTrueOnceTheTimePointPassed) {
  const SystemClock clock{};
  const InterruptSource source{};
  const auto time_point = clock.now() + 5ms;

  EXPECT_TRUE(clock.wait_until(time_point, source.token()));
  EXPECT_GE(clock.now(), time_point);
}

TEST(SystemClockTest, APastTimePointReturnsAtOnce) {
  const SystemClock clock{};
  const InterruptSource source{};

  EXPECT_TRUE(clock.wait_until(clock.now() - 1s, source.token()));
}

TEST(SystemClockTest, AnInterruptEndsTheWaitWithFalse) {
  const SystemClock clock{};
  bool reached{true};
  std::jthread waiter{[&clock, &reached](const std::stop_token& stop) {
    reached = clock.wait_until(clock.now() + 1h, InterruptToken{stop});
  }};

  waiter.request_stop();
  waiter.join();

  EXPECT_FALSE(reached);
}

TEST(SystemClockTest, AThreadWaitsAgainAfterAnInterruptedWait) {
  const SystemClock clock{};
  InterruptSource interrupted{};
  interrupted.interrupt();

  EXPECT_FALSE(clock.wait_until(clock.now() + 1h, interrupted.token()));

  const InterruptSource source{};
  const auto time_point = clock.now() + 5ms;
  EXPECT_TRUE(clock.wait_until(time_point, source.token()));
  EXPECT_GE(clock.now(), time_point);
}

TEST(FakeClockTest, NowReflectsSetNowAndAdvance) {
  FakeClock clock{};
  EXPECT_EQ(clock.now(), TimePoint{});

  clock.set_now(time_at(10s));
  EXPECT_EQ(clock.now(), time_at(10s));

  clock.advance(5s);
  EXPECT_EQ(clock.now(), time_at(15s));
}

TEST(FakeClockTest, AReachedTimePointReturnsAtOnce) {
  const FakeClock clock{time_at(10s)};
  const InterruptSource source{};

  EXPECT_TRUE(clock.wait_until(time_at(10s), source.token()));
}

TEST(FakeClockTest, AWaiterBlocksUntilAnAdvancePassesTheTimePoint) {
  FakeClock clock{};
  const InterruptSource source{};
  std::atomic<bool> returned{false};
  bool reached{false};
  std::jthread waiter{[&]() {
    reached = clock.wait_until(time_at(10s), source.token());
    returned = true;
  }};

  clock.advance(5s);
  std::this_thread::sleep_for(10ms);
  EXPECT_FALSE(returned);

  clock.advance(5s);
  waiter.join();
  EXPECT_TRUE(reached);
}

TEST(FakeClockTest, AnInterruptReleasesAWaiterWithFalse) {
  const FakeClock clock{};
  bool reached{true};
  std::jthread waiter{[&clock, &reached](const std::stop_token& stop) {
    reached = clock.wait_until(time_at(10s), InterruptToken{stop});
  }};

  waiter.request_stop();
  waiter.join();

  EXPECT_FALSE(reached);
}

} // namespace

// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// Pins the interrupt pair: a token follows its source, an interrupt is never
// reset, and a token adopted from a std::jthread follows that thread's stop
// request.

#include <stop_token>
#include <thread>

#include "xronos/physical_time/interrupt.hh"
#include "gtest/gtest.h"

namespace {

using xronos::physical_time::InterruptSource;
using xronos::physical_time::InterruptToken;

TEST(InterruptTest, ATokenFollowsItsSource) {
  InterruptSource source{};
  const auto token = source.token();
  EXPECT_FALSE(source.interrupted());
  EXPECT_FALSE(token.interrupted());

  source.interrupt();
  EXPECT_TRUE(source.interrupted());
  EXPECT_TRUE(token.interrupted());
}

TEST(InterruptTest, AnInterruptIsNeverReset) {
  InterruptSource source{};
  source.interrupt();
  source.interrupt();
  EXPECT_TRUE(source.token().interrupted());

  const InterruptSource fresh{};
  EXPECT_FALSE(fresh.token().interrupted());
}

TEST(InterruptTest, AnAdoptedTokenFollowsTheThreadsStopRequest) {
  bool interrupted{false};
  std::jthread thread{[&interrupted](const std::stop_token& stop) {
    const InterruptToken token{stop};
    while (!token.interrupted()) {
      std::this_thread::yield();
    }
    interrupted = true;
  }};

  thread.request_stop();
  thread.join();
  EXPECT_TRUE(interrupted);
}

} // namespace

// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/physical_time/physical_clock.hh"

#include <chrono>
#include <condition_variable>
#include <mutex>

#include "xronos/core/time.hh"
#include "xronos/physical_time/interrupt.hh"

namespace xronos::physical_time {

auto SystemClock::now() const noexcept -> core::TimePoint { return std::chrono::system_clock::now(); }

auto SystemClock::wait_until(core::TimePoint time_point, const InterruptToken& interrupt) const noexcept -> bool {
  // Using thread-local variables to avoid frequent allocation of
  // condition_variable_any's internal mutex. The mutex and lock do not guard
  // anything, but they are required by the condition variable. The predicate
  // is always false so that only the time point or the interrupt ends the
  // wait.
  thread_local std::mutex mutex{};
  thread_local std::condition_variable_any condition{};
  std::unique_lock<std::mutex> lock{mutex};
  condition.wait_until(lock, interrupt.stop_token(), time_point, [] { return false; });
  return !interrupt.interrupted();
}

} // namespace xronos::physical_time

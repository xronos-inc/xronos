// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_PHYSICAL_TIME_TESTING_FAKE_CLOCK_HH
#define XRONOS_PHYSICAL_TIME_TESTING_FAKE_CLOCK_HH

#include <condition_variable>
#include <mutex>

#include "xronos/core/time.hh"
#include "xronos/physical_time/interrupt.hh"
#include "xronos/physical_time/physical_clock.hh"

namespace xronos::physical_time {

// A clock that moves only when a test moves it. `wait_until` blocks until
// `set_now` or `advance` carries the time to the time point or the wait is
// interrupted, so a test decides exactly when a waiting run loop resumes.
class FakeClock final : public PhysicalClock {
public:
  FakeClock() = default;
  explicit FakeClock(core::TimePoint start)
      : now_{start} {}

  [[nodiscard]] auto now() const noexcept -> core::TimePoint final {
    std::scoped_lock lock{mutex_};
    return now_;
  }

  [[nodiscard]] auto wait_until(core::TimePoint time_point, const InterruptToken& interrupt) const noexcept
      -> bool final {
    std::unique_lock<std::mutex> lock{mutex_};
    return condition_.wait(lock, interrupt.stop_token(), [this, time_point] { return now_ >= time_point; });
  }

  void set_now(core::TimePoint time_point) noexcept {
    {
      std::scoped_lock lock{mutex_};
      now_ = time_point;
    }
    condition_.notify_all();
  }

  void advance(core::Duration duration) noexcept {
    {
      std::scoped_lock lock{mutex_};
      now_ += duration;
    }
    condition_.notify_all();
  }

private:
  mutable std::mutex mutex_{};
  mutable std::condition_variable_any condition_{};
  core::TimePoint now_{};
};

} // namespace xronos::physical_time

#endif // XRONOS_PHYSICAL_TIME_TESTING_FAKE_CLOCK_HH

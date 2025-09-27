// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_IMPL_TIME_BARRIER_HH
#define XRONOS_RUNTIME_DEFAULT_IMPL_TIME_BARRIER_HH

#include <atomic>
#include <functional>
#include <mutex>

#include "xronos/runtime/default/impl/fwd.hh"
#include "xronos/runtime/default/impl/logical_time.hh"
#include "xronos/runtime/default/impl/scheduler.hh"
#include "xronos/runtime/default/impl/time.hh"

namespace xronos::runtime::default_::impl {

class PhysicalTimeBarrier {
  inline static std::atomic<Duration> last_observed_physical_time_{Duration::zero()};

public:
  static auto try_acquire_tag(const Tag& tag) -> bool {
    // First, we compare against the last observed physical time. This variable
    // serves as a cache for reading the physical clock. Reading from the physical
    // clock can be slow and, thus, this is an optimization that ensures that we
    // only read the clock when it is needed.
    if (tag.time_point().time_since_epoch() < last_observed_physical_time_.load(std::memory_order_acquire)) {
      return true;
    }

    auto physical_time = get_physical_time();
    last_observed_physical_time_.store(physical_time.time_since_epoch(), std::memory_order_release);

    return tag.time_point() < physical_time;
  }

  static auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock, Scheduler* scheduler,
                          const std::function<bool(void)>& abort_waiting) -> bool {
    if (try_acquire_tag(tag)) {
      return true;
    }
    return !scheduler->wait_until(lock, tag.time_point(), abort_waiting);
  }
};

} // namespace xronos::runtime::default_::impl

#endif // XRONOS_RUNTIME_DEFAULT_IMPL_TIME_BARRIER_HH

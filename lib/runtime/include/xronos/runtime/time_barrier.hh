// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_TIME_BARRIER_HH
#define XRONOS_RUNTIME_TIME_BARRIER_HH

#include "fwd.hh"
#include "logical_time.hh"
#include "scheduler.hh"
#include "time.hh"
#include <functional>

namespace xronos::runtime {

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

class LogicalTimeBarrier {
private:
  LogicalTime released_time_;
  Scheduler* scheduler_;

  /*
   * Note that we use the main scheduler lock to protect our `released_time_`
   * variable. We cannot use a local mutex here that only protects our local
   * state. This is, because we also need to use a lock to wait on a condition
   * variable. This lock must be the same lock as the one that is used when
   * updating the wait condition. (See "An atomic predicate" in
   * https://www.modernescpp.com/index.php/c-core-guidelines-be-aware-of-the-traps-of-condition-variables)
   *
   * Here we have two conditions in which we want to stop the wait:
   *   1. When the tag that we are waiting for was released.
   *   2. When the given function `abort_waiting` returns true. In a typical usage
   *      scenario, this would be the case when a new event was scheduled.
   *
   * We need to make sure that both conditions are updated while holding the
   * lock that we use for waiting on the condition variable. Our only option for
   * this is to use the global scheduler lock.
   */
public:
  LogicalTimeBarrier(Scheduler* scheduler)
      : scheduler_(scheduler) {}

  void release_tag(const LogicalTime& tag) {
    auto lock = scheduler_->lock();
    released_time_.advance_to(tag);
  }

  // The caller must hold a lock on the scheduler mutex
  auto try_acquire_tag(const Tag& tag) { return tag <= released_time_; }

  auto acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                   const std::function<bool(void)>& abort_waiting) -> bool {
    if (try_acquire_tag(tag)) {
      return true;
    }
    scheduler_->wait(lock, [this, &tag, &abort_waiting]() { return try_acquire_tag(tag) || abort_waiting(); });
    return !abort_waiting();
  }
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_TIME_BARRIER_HH

// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_IMPL_ACTION_IMPL_HH
#define XRONOS_RUNTIME_IMPL_ACTION_IMPL_HH

#include "xronos/runtime/assert.hh"
#include "xronos/runtime/environment.hh"
#include "xronos/runtime/reactor.hh"

#include <iterator>
#include <mutex>

namespace xronos::runtime {

template <class T> template <class Dur> void Action<T>::schedule(const ImmutableValuePtr<T>& value_ptr, Dur delay) {
  Duration time_delay = std::chrono::duration_cast<Duration>(delay);
  validate(time_delay >= Duration::zero(), "Schedule cannot be called with a negative delay!");
  validate(value_ptr != nullptr, "Actions may not be scheduled with a nullptr value!");

  auto* scheduler = environment().scheduler();
  if (is_logical()) {
    time_delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(time_delay);

    scheduler->schedule_sync(this, tag);
    events_[tag] = value_ptr;
  } else {
    std::lock_guard<std::mutex> lock{mutex_events_};
    // We must call schedule_async while holding the mutex, because otherwise
    // the scheduler could already start processing the event that we schedule
    // and call setup() on this action before we insert the value in events_.
    // Holding both the local mutex mutex_events_ and the scheduler mutex (in
    // schedule async) should not lead to a deadlock as the scheduler will
    // only hold one of the two mutexes at once.
    auto tag = scheduler->schedule_async(this, time_delay);
    events_[tag] = value_ptr;
  }
}

template <class Dur> void Action<void>::schedule(Dur delay) {
  Duration time_delay = std::chrono::duration_cast<Duration>(delay);
  validate(time_delay >= Duration::zero(), "Schedule cannot be called with a negative delay!");
  auto* scheduler = environment().scheduler();
  if (is_logical()) {
    time_delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(time_delay);
    scheduler->schedule_sync(this, tag);
  } else {
    scheduler->schedule_async(this, time_delay);
  }
}

template <class T> auto Action<T>::schedule_at(const ImmutableValuePtr<T>& value_ptr, const Tag& tag) -> bool {
  validate(value_ptr != nullptr, "Actions may not be scheduled with a nullptr value!");

  auto* scheduler = environment().scheduler();
  if (is_logical()) {
    if (tag <= scheduler->logical_time()) {
      return false;
    }

    scheduler->schedule_sync(this, tag);
    events_[tag] = value_ptr;
  } else {
    std::lock_guard<std::mutex> lock{mutex_events_};
    // We must call schedule_async while holding the mutex, because otherwise
    // the scheduler could already start processing the event that we schedule
    // and call setup() on this action before we insert the value in events_.
    // Holding both the local mutex mutex_events_ and the scheduler mutex (in
    // schedule async) should not lead to a deadlock as the scheduler will
    // only hold one of the two mutexes at once.
    bool result = scheduler->schedule_async_at(this, tag);
    if (result) {
      events_[tag] = value_ptr;
    }
    return result;
  }
  return true;
}

template <class T> void Action<T>::setup() noexcept {
  BaseAction::setup();
  if (value_ptr_ == nullptr) { // only do this once, even if the action was triggered multiple times
    // lock if this is a physical action
    std::unique_lock<std::mutex> lock =
        is_logical() ? std::unique_lock<std::mutex>() : std::unique_lock<std::mutex>(mutex_events_);
    const auto& node = events_.extract(events_.begin());
    reactor_assert(!node.empty());
    reactor_assert(node.key() == environment().scheduler()->logical_time());
    value_ptr_ = std::move(node.mapped());
  }
  reactor_assert(value_ptr_ != nullptr);
}

template <class T> void Action<T>::cleanup() noexcept {
  BaseAction::cleanup();
  value_ptr_ = nullptr;
}

template <class T>
PhysicalAction<T>::PhysicalAction(std::string_view name, Reactor& container)
    : Action<T>(name, container, false, Duration::zero()) {
  // all physical actions act as input actions to the program as they can be
  // scheduled from external threads
  this->environment().register_input_action(*this);
}

} // namespace xronos::runtime

#endif

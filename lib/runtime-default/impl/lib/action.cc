// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2019 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/impl/action.hh"

#include <any>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/assert.hh"
#include "xronos/runtime/default/impl/environment.hh"
#include "xronos/runtime/default/impl/logical_time.hh"
#include "xronos/runtime/default/impl/reaction.hh"
#include "xronos/runtime/default/impl/reactor.hh"
#include "xronos/runtime/default/impl/time.hh"
#include "xronos/runtime/default/impl/time_barrier.hh"
#include "xronos/util/assert.hh"

namespace xronos::runtime::default_::impl {

void BaseAction::register_trigger(Reaction* reaction) {
  util::assert_(reaction != nullptr);
  util::assert_(&this->environment() == &reaction->environment());
  validate_phase(this, Phase::Construction);
  validate(this->container() == reaction->container(),
           "Action triggers must belong to the same reactor as the triggered "
           "reaction");
  [[maybe_unused]] bool result = triggers_.insert(reaction).second;
  util::assert_(result);
}

void BaseAction::register_scheduler(Reaction* reaction) {
  util::assert_(reaction != nullptr);
  util::assert_(&this->environment() == &reaction->environment());
  validate_phase(this, Phase::Construction);
  // the reaction must belong to the same reactor as this action
  validate(this->container() == reaction->container(), "Scheduable actions must belong to the same reactor as the "
                                                       "triggered reaction");
  [[maybe_unused]] bool result = schedulers_.insert(reaction).second;
  util::assert_(result);
}

void Timer::startup() {
  if (period_ <= Duration::zero()) {
    throw std::runtime_error("Timer period must be greater than zero.");
  }
  if (offset_ < Duration::zero()) {
    throw std::runtime_error("Timer offset must be greater than or equal to zero.");
  }

  const Tag& start_tag = environment().start_tag();
  if (offset_ != Duration::zero()) {
    environment().scheduler()->schedule_sync(this, start_tag.delay(offset_));
  } else {
    environment().scheduler()->schedule_sync(this, start_tag);
  }
}

void Timer::cleanup() noexcept {
  BaseAction::cleanup();
  // schedule the timer again
  Tag now = Tag::from_logical_time(environment().logical_time());
  Tag next = now.delay(period_);
  environment().scheduler()->schedule_sync(this, next);
}

void Timer::set_offset(Duration offset) {
  if (this->environment().phase() != Phase::Construction) {
    throw std::runtime_error("The offset may only be set during program initialization.");
  }
  this->offset_ = offset;
}
void Timer::set_period(Duration period) {
  if (this->environment().phase() != Phase::Construction) {
    throw std::runtime_error("The offset may only be set during program initialization.");
  }
  this->period_ = period;
}

void StartupTrigger::startup() {
  const Tag& start_tag = environment().start_tag();
  environment().scheduler()->schedule_sync(this, start_tag);
}

void ShutdownTrigger::startup() {
  auto timeout = environment().timeout();

  if (timeout >= Duration::zero() && timeout != Duration::max()) {
    const Tag& start_tag = environment().start_tag();
    environment().scheduler()->schedule_sync(this, start_tag.delay(timeout));
  }
}

void ShutdownTrigger::shutdown() {
  Tag tag = Tag::from_logical_time(environment().logical_time()).delay();
  environment().scheduler()->schedule_sync(this, tag);
}

auto Action::schedule_at(const std::any& value, const Tag& tag) -> bool {
  util::assert_(value.has_value());
  auto* scheduler = environment().scheduler();
  if (is_logical()) {
    if (tag <= scheduler->logical_time()) {
      return false;
    }
    scheduler->schedule_sync(this, tag);
    events_[tag] = value;
  } else {
    // We must call schedule_async while holding the mutex, because otherwise
    // the scheduler could already start processing the event that we schedule
    // and call setup() on this action before we insert the value in events_.
    // Holding both the local mutex mutex_events_ and the scheduler mutex (in
    // schedule async) should not lead to a deadlock as the scheduler will
    // only hold one of the two mutexes at once.
    bool result = scheduler->schedule_async_at(this, tag);
    if (result) {
      events_[tag] = value;
    }
    return result;
  }
  return true;
}

void Action::schedule(const std::any& value, Duration delay) noexcept {
  util::assert_(delay >= Duration::zero());
  util::assert_(value.has_value());

  auto* scheduler = environment().scheduler();
  if (is_logical()) {
    delay += this->min_delay();
    auto tag = Tag::from_logical_time(scheduler->logical_time()).delay(delay);

    scheduler->schedule_sync(this, tag);
    events_[tag] = value;
  } else {
    std::lock_guard<std::mutex> lock{mutex_events_};
    // We must call schedule_async while holding the mutex, because otherwise
    // the scheduler could already start processing the event that we schedule
    // and call setup() on this action before we insert the value in events_.
    // Holding both the local mutex mutex_events_ and the scheduler mutex (in
    // schedule async) should not lead to a deadlock as the scheduler will
    // only hold one of the two mutexes at once.
    auto tag = scheduler->schedule_async(this, delay);
    events_[tag] = value;
  }
}

auto BaseAction::acquire_tag(const Tag& tag, std::unique_lock<std::mutex>& lock,
                             const std::function<bool(void)>& abort_waiting) -> bool {
  util::assert_(!logical_);
  util::assert_(lock.owns_lock());
  return PhysicalTimeBarrier::acquire_tag(tag, lock, environment().scheduler(), abort_waiting);
}

void Action::setup() noexcept {
  BaseAction::setup();
  if (!current_value_.has_value()) { // only do this once, even if the action was triggered multiple times
    // lock if this is a physical action
    std::unique_lock<std::mutex> lock =
        is_logical() ? std::unique_lock<std::mutex>() : std::unique_lock<std::mutex>(mutex_events_);
    const auto& node = events_.extract(events_.begin());
    util::assert_(!node.empty());
    util::assert_(node.key() == environment().scheduler()->logical_time());
    current_value_ = std::move(node.mapped());
  }
  util::assert_(current_value_.has_value());
}

void Action::cleanup() noexcept {
  BaseAction::cleanup();
  current_value_.reset();
}

PhysicalAction::PhysicalAction(const core::Element& element_info, Reactor& container)
    : Action(element_info, container, false, Duration::zero()) {
  // all physical actions act as input actions to the program as they can be
  // scheduled from external threads
  this->environment().register_input_action(*this);
}

Timer::Timer(const core::Element& element_info, Reactor& container)
    : BaseAction(element_info, container, true, Duration::zero())
    , offset_(std::get<core::PeriodicTimerTag>(element_info.type).properties->offset)
    , period_(std::get<core::PeriodicTimerTag>(element_info.type).properties->period) {}

} // namespace xronos::runtime::default_::impl

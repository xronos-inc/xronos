// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/detail/scheduler.hh"

#include <algorithm>
#include <any>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <mutex>
#include <ranges>
#include <variant>
#include <vector>

#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/logical_time/tag.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::runtime::default_::detail {

void EventQueue::schedule_event(std::uint64_t element_uid, const std::any& value, const logical_time::Tag& tag) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    event_queue_[tag].emplace_back(value, element_uid);
  }
  cv_.notify_one();
}

void EventQueue::trigger_external_event(std::uint64_t element_uid, const std::any& value) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    core::TimePoint now{std::chrono::system_clock::now()};
    if (now <= last_external_event_timestamp_) {
      // On some platforms with low clock resolution we might get the same
      // timestamp for consecutive calls to now(). If we were to use the same
      // tag twice, for the same element, we would overwrite the event.
      // Therefore, we increment the timestamp by one ns.
      now = last_external_event_timestamp_ + std::chrono::nanoseconds{1};
    }
    logical_time::Tag tag{now, 0};
    last_external_event_timestamp_ = now;

    event_queue_[tag].emplace_back(value, element_uid);
  }
  cv_.notify_one();
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
auto EventQueue::wait_until_next_event(const logical_time::Tag& max_tag)
    -> std::map<logical_time::Tag, std::vector<Event>>::node_type {
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    if (event_queue_.empty() || event_queue_.begin()->first >= max_tag) {
      if (!has_external_triggers_) {
        return decltype(event_queue_)::node_type{};
      }
      auto status = cv_.wait_until(lock, max_tag.timestamp());
      if (event_queue_.empty() || event_queue_.begin()->first >= max_tag) {
        if (status == std::cv_status::timeout && (event_queue_.empty() || event_queue_.begin()->first >= max_tag)) {
          return decltype(event_queue_)::node_type{};
        }
        // If we reach here, we woke up spuriously or because a new event got
        // inserted into the queue. We simply go back to the start of the loop
        // and try again.
      }
    } else {
      if (fast_mode_ && !has_external_triggers_) {
        return event_queue_.extract(event_queue_.begin());
      }

      auto next_tag = event_queue_.begin()->first;
      auto status = cv_.wait_until(lock, next_tag.timestamp());
      if (status == std::cv_status::timeout && next_tag == event_queue_.begin()->first) {
        return event_queue_.extract(event_queue_.begin());
      }
      // If we reach here, we woke up spuriously or because a new event got
      // inserted into the queue. We simply go back to the start of the loop
      // and try again.
    }
  }
}

auto EventQueue::check_has_events_at_tag(const logical_time::Tag& tag) const noexcept -> bool {
  std::unique_lock<std::mutex> lock(mutex_);
  return !event_queue_.empty() && event_queue_.begin()->first == tag;
}

void Scheduler::init(const core::ReactorModel& reactor_model, const RuntimeModel& runtime_model) {
  reactor_model_ = &reactor_model;
  runtime_model_ = &runtime_model;

  for (auto index : std::views::iota(0U, runtime_model.ordered_reaction_uids.size())) {
    auto res = reactions_.try_emplace(
        runtime_model.ordered_reaction_uids[index], index, runtime_model.ordered_reaction_uids[index],
        core::get_properties<core::ReactionTag>(
            reactor_model.element_registry.get(runtime_model.ordered_reaction_uids[index]))
            .handler);
    util::assert_(res.second);
  }

  if (!reactor_model.element_registry.elements_of_type<core::PhysicalEventTag>().empty()) {
    event_queue_.set_has_external_triggers();
    has_external_triggers_ = true;
  }
}

void Scheduler::schedule_event(std::uint64_t element_uid, const std::any& value, core::Duration delay) {
  schedule_event(element_uid, value, current_tag_ + delay);
}

void Scheduler::schedule_event(std::uint64_t element_uid, const std::any& value, const logical_time::Tag& tag) {
  util::log::debug() << "Scheduling new event for element " << reactor_model_->element_registry.get(element_uid).fqn
                     << " at tag " << tag << '.';
  event_queue_.schedule_event(element_uid, value, tag);
}

void Scheduler::trigger_event(std::uint64_t element_uid, const std::any& value) {
  auto& event_source = event_sources_[element_uid];

  event_source.activate(value);
  active_events_.push_back(element_uid);

  auto dbg = util::log::debug();
  dbg << "Triggering element " << reactor_model_->element_registry.get(element_uid).fqn
      << ". This triggers the following reactions:\n";

  auto it = runtime_model_->triggers.find(element_uid);
  if (it != runtime_model_->triggers.end()) {
    for (auto reaction_uid : it->second.triggered_reaction_uids) {
      dbg << "  - " << reactor_model_->element_registry.get(element_uid).fqn << '\n';
      util::assert_(reactions_.contains(reaction_uid));
      ready_queue_.emplace_back(reactions_.at(reaction_uid));
    }
  }
}

void Scheduler::set_port(std::uint64_t port_uid, const std::any& value) {
  util::log::debug() << "Setting port " << reactor_model_->element_registry.get(port_uid).fqn;
  util::assert_(event_sources_.contains(port_uid));
  trigger_event(port_uid, value);

  auto connections_it = runtime_model_->end_to_end_connections.find(port_uid);
  if (connections_it != runtime_model_->end_to_end_connections.end()) {
    for (const auto& properties : connections_it->second) {
      if (properties.delay.has_value()) {
        schedule_event(properties.to_uid, value, *properties.delay);
      } else {
        trigger_event(properties.to_uid, value);
      }
    }
  }
}

void Scheduler::execute() {
  start_tag_ = logical_time::Tag{std::chrono::system_clock::now(), 0};
  current_tag_ = start_tag_;
  shutdown_tag_ = start_tag_ + execution_properties_.timeout;

  util::log::debug() << "Scheduler starts execution at tag " << current_tag_;

  // Trigger all startup events and timers
  for (auto startup_uid : runtime_model_->startup_trigger_uids) {
    schedule_event(startup_uid, std::monostate{}, start_tag_);
  }
  for (const auto& [timer_uid, properties] : runtime_model_->periodic_timer_properties) {
    if (properties.offset == core::Duration::zero()) {
      schedule_event(timer_uid, std::monostate{}, start_tag_);
    } else {
      schedule_event(timer_uid, std::monostate{}, properties.offset);
    }
  }

  // Process all events until we reach the shutdown tag or run out of events
  while (process_next_tag(shutdown_tag_)) {
  }

  util::log::debug() << "Triggering all shutdown events.";

  if (!has_external_triggers_ && !event_queue_.check_has_events_at_tag(shutdown_tag_)) {
    shutdown_tag_ = current_tag_ + core::Duration::zero();
  }

  // Trigger the shutdown events
  for (auto shutdown_uid : runtime_model_->shutdown_trigger_uids) {
    schedule_event(shutdown_uid, std::monostate{}, shutdown_tag_);
  }

  // Process shutdown reactions
  process_next_tag(shutdown_tag_ + core::Duration::zero());

  util::log::debug() << "Scheduler is done executing.";

  if (active_exception_ != nullptr) {
    std::rethrow_exception(active_exception_);
  }
}

auto Scheduler::process_next_tag(const logical_time::Tag& max_tag) -> bool {
  auto event_handle = event_queue_.wait_until_next_event(max_tag);
  if (event_handle.empty()) {
    return false;
  }

  current_tag_ = event_handle.key();

  util::log::debug() << "Advance logical time to tag " << current_tag_ << " and start processing reactions.";

  for (const auto& event : event_handle.mapped()) {
    trigger_event(event.element_uid, event.value);
  }
  execute_all_ready_reactions();

  reset_all_active_events();

  util::log::debug() << "Done processing tag " << current_tag_ << '.';
  return true;
}

void Scheduler::execute_all_ready_reactions() {
  while (!ready_queue_.empty()) {
    std::ranges::sort(ready_queue_);
    auto [erase_begin, erase_end] = std::ranges::unique(ready_queue_);
    ready_queue_.erase(erase_begin, erase_end);

    auto& handler = ready_queue_.front().handler.get();
    execute_reaction(handler);

    ready_queue_.pop_front();
  }
}

void Scheduler::reset_all_active_events() {
  for (auto uid : active_events_) {
    event_sources_[uid].reset();
    if (auto it = runtime_model_->periodic_timer_properties.find(uid);
        it != runtime_model_->periodic_timer_properties.end()) {
      schedule_event(uid, std::monostate{}, it->second.period);
    }
  }
  active_events_.clear();
}

void Scheduler::execute_reaction(const std::function<void()>& handler) {
  try {
    handler();
  } catch (...) {
    const auto& fqn = reactor_model_->element_registry.get(ready_queue_.front().uid).fqn;
    util::log::error() << "Exception caught during execution of reaction " << fqn << ". Shutting down.";
    trigger_shutdown();
    if (active_exception_ == nullptr) {
      active_exception_ = std::current_exception();
    } else {
      util::log::warn() << "Dropping exception details as there already is an active exception.";
    }
  }
}

} // namespace xronos::runtime::default_::detail

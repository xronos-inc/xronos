// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/detail/scheduler.hh"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/abi/types.hh"
#include "xronos/abi/value.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/logical_time/tag.hh"
#include "xronos/physical_time/interrupt.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"
#include "xronos/value/boxing.hh"

namespace xronos::runtime::default_::detail {

namespace {

// Backs the ABI's sink-based serializer with an owned buffer the
// deserializing ports then deserialize from.
class VectorByteSink final : public abi::ByteSink {
public:
  void write(const std::byte* data, std::size_t size) final {
    auto bytes = std::span{data, size};
    buffer_.insert(buffer_.end(), bytes.begin(), bytes.end());
  }
  [[nodiscard]] auto take() && -> std::vector<std::byte> { return std::move(buffer_); }

private:
  std::vector<std::byte> buffer_{};
};

} // namespace

void EventQueue::schedule_event(std::uint64_t element_uid, abi::AnyValue&& value, const logical_time::Tag& tag) {
  // Every caller runs on the scheduler thread, so no wait in next_event is
  // in progress and none needs waking. next_event relies on this invariant.
  std::scoped_lock lock(mutex_);
  event_queue_[tag].emplace_back(std::move(value), element_uid);
}

auto EventQueue::trigger_physical_event(std::uint64_t element_uid, abi::AnyValue&& value) -> abi::TriggerStatus {
  std::scoped_lock lock(mutex_);
  if (!started_) {
    return abi::TriggerStatus::NotStarted;
  }
  core::TimePoint now{clock_.get().now()};
  if (now <= last_physical_event_timestamp_) {
    // On some platforms with low clock resolution we might get the same
    // timestamp for consecutive calls to now(). If we were to use the same
    // tag twice, for the same element, we would overwrite the event.
    // Therefore, we increment the timestamp by one ns.
    now = last_physical_event_timestamp_ + std::chrono::nanoseconds{1};
  }
  logical_time::Tag tag{now, 0};
  if (stopped_ || stop_requested_ || tag >= shutdown_bound_) {
    return abi::TriggerStatus::Stopped;
  }
  last_physical_event_timestamp_ = now;

  event_queue_[tag].emplace_back(std::move(value), element_uid);
  wait_interrupt_.interrupt();
  return abi::TriggerStatus::Accepted;
}

auto EventQueue::next_event(const logical_time::Tag& max_tag, bool stoppable)
    -> std::map<logical_time::Tag, std::vector<Event>>::node_type {
  // The next tag when it lies before max_tag.
  auto next_tag_before_max = [this, &max_tag]() -> std::optional<logical_time::Tag> {
    if (event_queue_.empty() || event_queue_.begin()->first >= max_tag) {
      return std::nullopt;
    }
    return event_queue_.begin()->first;
  };

  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    // Every wait below ends on an interrupt and loops back here, so a stop
    // reaches a blocked call promptly.
    if (stoppable && stop_requested_) {
      return decltype(event_queue_)::node_type{};
    }
    auto next_tag = next_tag_before_max();
    if (!next_tag.has_value() && !has_physical_events_) {
      return decltype(event_queue_)::node_type{};
    }
    if (next_tag.has_value() && fast_mode_ && !has_physical_events_) {
      return event_queue_.extract(event_queue_.begin());
    }

    // Wait for the physical clock to reach the next tag, or max_tag when no
    // tag lies before it. Only two calls from other threads end the wait
    // early: trigger_physical_event, which may add an earlier tag, and
    // request_stop. Nothing else changes the queue meanwhile, because
    // schedule_event runs only on this thread. Both call interrupt() on
    // wait_interrupt_. An interrupt is never reset, so each wait takes a
    // fresh source; a reused one would end every later wait at once.
    wait_interrupt_ = physical_time::InterruptSource{};
    auto interrupt = wait_interrupt_.token();
    lock.unlock();
    bool reached = clock_.get().wait_until(next_tag.value_or(max_tag).timestamp(), interrupt);
    lock.lock();

    // A wait that ended early, or one during which the next tag changed, is
    // re-evaluated from the top.
    if (!reached || next_tag_before_max() != next_tag) {
      continue;
    }
    if (!next_tag.has_value()) {
      return decltype(event_queue_)::node_type{};
    }
    return event_queue_.extract(event_queue_.begin());
  }
}

void EventQueue::discard_events_at_tag(const logical_time::Tag& tag) noexcept {
  std::scoped_lock lock(mutex_);
  event_queue_.erase(tag);
}

void Scheduler::init(const core::ReactorModel& reactor_model, const RuntimeModel& runtime_model) {
  reactor_model_ = &reactor_model;
  runtime_model_ = &runtime_model;

  for (auto index : std::views::iota(0U, runtime_model.ordered_reaction_uids.size())) {
    auto uid = runtime_model.ordered_reaction_uids.at(index);
    auto& properties = core::get_properties<core::ReactionTag>(reactor_model.element_registry.get(uid));
    // Guaranteed by validator::check_reaction_handlers on validated models.
    util::assert_(properties.handler != nullptr);
    auto res = reactions_.try_emplace(uid, index, uid, *properties.handler);
    util::assert_(res.second);
  }

  if (!reactor_model.element_registry.elements_of_type<core::PhysicalEventTag>().empty()) {
    event_queue_.set_has_physical_events();
    has_physical_events_ = true;
  }
}

void Scheduler::schedule_event(std::uint64_t element_uid, abi::AnyValue&& value, core::Duration delay) {
  schedule_event(element_uid, std::move(value), current_tag_ + delay);
}

void Scheduler::schedule_event(std::uint64_t element_uid, abi::AnyValue&& value, const logical_time::Tag& tag) {
  util::log::debug() << "Scheduling new event for element " << reactor_model_->element_registry.get(element_uid).fqn
                     << " at tag " << tag << '.';
  event_queue_.schedule_event(element_uid, std::move(value), tag);
}

void Scheduler::trigger_event(std::uint64_t element_uid, abi::AnyValue&& value) {
  auto& present_value = present_values_[element_uid];

  present_value.set(std::move(value));
  present_elements_.push_back(element_uid);

  auto dbg = util::log::debug();
  dbg << "Triggering element " << reactor_model_->element_registry.get(element_uid).fqn
      << ". This triggers the following reactions:\n";

  auto it = runtime_model_->triggers.find(element_uid);
  if (it != runtime_model_->triggers.end()) {
    for (auto reaction_uid : it->second.triggered_reaction_uids) {
      dbg << "  - " << reactor_model_->element_registry.get(element_uid).fqn << '\n';
      util::assert_(reactions_.contains(reaction_uid));
      triggered_reactions_.emplace_back(reactions_.at(reaction_uid));
    }
  }
}

void Scheduler::set_port(std::uint64_t port_uid, abi::AnyValue&& value) {
  util::log::debug() << "Setting port " << reactor_model_->element_registry.get(port_uid).fqn;
  util::assert_(present_values_.contains(port_uid));

  // The port's trigger and effect share one present value, so triggering the
  // port stores the value for the writing reaction to read back and
  // triggers the reactions that observe the port. An origin port whose
  // connection tree keeps no delivery has no entry in the map.
  auto tree_it = runtime_model_->connection_trees.find(port_uid);
  if (tree_it == runtime_model_->connection_trees.end()) {
    trigger_event(port_uid, std::move(value));
    return;
  }

  trigger_event(port_uid, abi::AnyValue{value});
  (void)deliver_segment(tree_it->second, std::move(value));
}

auto Scheduler::deliver_segment(const ResolvedConnectionSegment& segment, abi::AnyValue&& value) noexcept -> bool {
  // Fanning out needs one box per receiver; only the last delivery can steal
  // the incoming value, and only when no serialization still needs it.
  auto deliver = [this](const core::ReceivingPort& delivery, abi::AnyValue&& to_send) {
    if (delivery.delay.has_value()) {
      schedule_event(delivery.port_uid, std::move(to_send), *delivery.delay);
    } else {
      trigger_event(delivery.port_uid, std::move(to_send));
    }
  };
  for (std::size_t index = 0; index < segment.deliveries.size(); index++) {
    if (index + 1 == segment.deliveries.size() && segment.serializations.empty()) {
      deliver(segment.deliveries.at(index), std::move(value));
    } else {
      deliver(segment.deliveries.at(index), abi::AnyValue{value});
    }
  }

  for (const auto& serialization : segment.serializations) {
    // Encode once per serializing port, however many deserializing ports
    // the crossing has.
    std::vector<std::byte> bytes{};
    try {
      VectorByteSink sink{};
      serialization.serializer->serialize(value, sink);
      bytes = std::move(sink).take();
    } catch (...) {
      handle_boundary_error(serialization.serialize_uid, serialization.deserializations.front().deserialize_uid, true);
      return false;
    }
    for (const auto& deserialization : serialization.deserializations) {
      // Each deserializing port delivers the box created by its own
      // deserializer, so the receivers never share the sender's box.
      abi::AnyValue decoded{};
      try {
        decoded = deserialization.deserializer->deserialize(bytes.data(), bytes.size());
      } catch (...) {
        handle_boundary_error(serialization.serialize_uid, deserialization.deserialize_uid, false);
        return false;
      }
      if (!deliver_segment(deserialization.segment, std::move(decoded))) {
        return false;
      }
    }
  }
  return true;
}

void Scheduler::handle_boundary_error(std::uint64_t serialize_uid, std::uint64_t deserialize_uid,
                                      bool serializing) noexcept {
  const auto& registry = reactor_model_->element_registry;
  std::string message{serializing ? "Failed to serialize" : "Failed to deserialize"};
  message += " a value crossing the cross-boundary connection from ";
  message += registry.get(serialize_uid).fqn;
  message += " to ";
  message += registry.get(deserialize_uid).fqn;
  try {
    throw;
  } catch (const std::exception& error) {
    message += ": ";
    message += error.what();
  } catch (...) {
  }
  util::log::error() << message << " Shutting down.";
  abort_execution_ = true;
  trigger_shutdown();
  if (active_exception_ == nullptr) {
    active_exception_ = std::current_exception();
  } else {
    util::log::error() << "Dropping this exception because an earlier exception is already recorded.";
  }
}

void Scheduler::execute() {
  start_tag_ = logical_time::Tag{clock_.get().now(), 0};
  current_tag_ = start_tag_;
  // The timeout bounds execution by `start + timeout` inclusive. An event at
  // exactly the timeout gets its own tag; shutdown follows one microstep
  // later.
  shutdown_bound_ = (start_tag_ + execution_properties_.timeout).increment_microstep();

  event_queue_.mark_started(shutdown_bound_);
  started_.store(true, std::memory_order_release);
  // Guards against an early exit (e.g., due to an exception) and ensures that
  // the scheduler is marked as stopped.
  struct StopGuard {
    explicit StopGuard(EventQueue& queue)
        : queue_{queue} {}
    StopGuard(const StopGuard&) = delete;
    StopGuard(StopGuard&&) = delete;
    auto operator=(const StopGuard&) = delete;
    auto operator=(StopGuard&&) = delete;
    ~StopGuard() { queue_.get().mark_stopped(); }

  private:
    std::reference_wrapper<EventQueue> queue_;
  } stop_guard{event_queue_};

  util::log::debug() << "Scheduler starts execution at tag " << current_tag_;

  // Trigger all startup events and timers
  for (auto startup_uid : runtime_model_->startup_trigger_uids) {
    schedule_event(startup_uid, xronos::value::make<abi::Void>(), start_tag_);
  }
  for (const auto& [timer_uid, properties] : runtime_model_->periodic_timer_properties) {
    if (properties.offset == core::Duration::zero()) {
      schedule_event(timer_uid, xronos::value::make<abi::Void>(), start_tag_);
    } else {
      schedule_event(timer_uid, xronos::value::make<abi::Void>(), properties.offset);
    }
  }

  // Process all events until we reach the shutdown bound, run out of
  // events, or a stop request ends the loop early.
  while (process_next_tag(shutdown_bound_)) {
  }

  util::log::debug() << "Triggering all shutdown events.";

  // The shutdown tag is one microstep after the last processed tag instead
  // of the shutdown bound. With a physical event this is unsound, since an
  // event may still arrive before that tag; after a stop request the
  // remaining events are abandoned deliberately.
  auto shutdown_tag = shutdown_bound_;
  if (!has_physical_events_ || event_queue_.stop_requested()) {
    shutdown_tag = current_tag_ + core::Duration::zero();
  }

  // Only shutdown events run at the shutdown tag. Any other event landing
  // there, such as one scheduled with zero delay by the reaction that
  // requested shutdown, is dropped.
  event_queue_.discard_events_at_tag(shutdown_tag);

  // Trigger the shutdown events
  for (auto shutdown_uid : runtime_model_->shutdown_trigger_uids) {
    schedule_event(shutdown_uid, xronos::value::make<abi::Void>(), shutdown_tag);
  }

  process_shutdown_tag(shutdown_tag);

  util::log::debug() << "Scheduler is done executing.";

  if (active_exception_ != nullptr) {
    std::rethrow_exception(active_exception_);
  }
}

auto Scheduler::process_next_tag(const logical_time::Tag& max_tag) -> bool {
  return process_events(event_queue_.wait_until_next_event(max_tag));
}

void Scheduler::process_shutdown_tag(const logical_time::Tag& shutdown_tag) {
  // The bound sits one microstep past the shutdown tag, so the wait extracts
  // the events at the shutdown tag itself.
  process_events(event_queue_.wait_for_shutdown_events(shutdown_tag + core::Duration::zero()));
}

auto Scheduler::process_events(std::map<logical_time::Tag, std::vector<Event>>::node_type event_handle) -> bool {
  if (event_handle.empty()) {
    return false;
  }

  current_tag_ = event_handle.key();

  util::log::debug() << "Advance logical time to tag " << current_tag_ << " and start processing reactions.";

  for (auto& event : event_handle.mapped()) {
    trigger_event(event.element_uid, std::move(event.value));
  }
  execute_triggered_reactions();

  clear_present_values();

  util::log::debug() << "Done processing tag " << current_tag_ << '.';
  return true;
}

void Scheduler::execute_triggered_reactions() {
  while (!triggered_reactions_.empty() && !abort_execution_) {
    std::ranges::sort(triggered_reactions_);
    auto [erase_begin, erase_end] = std::ranges::unique(triggered_reactions_);
    triggered_reactions_.erase(erase_begin, erase_end);

    auto& handler = triggered_reactions_.front().handler.get();
    execute_reaction(handler);

    triggered_reactions_.pop_front();
  }
  abort_execution_ = false;
  triggered_reactions_.clear();
}

void Scheduler::clear_present_values() {
  for (auto uid : present_elements_) {
    present_values_[uid].clear();
    if (auto it = runtime_model_->periodic_timer_properties.find(uid);
        it != runtime_model_->periodic_timer_properties.end()) {
      schedule_event(uid, xronos::value::make<abi::Void>(), it->second.period);
    }
  }
  present_elements_.clear();
}

void Scheduler::execute_reaction(abi::ReactionHandler& handler) {
  try {
    handler.invoke();
  } catch (...) {
    const auto& fqn = reactor_model_->element_registry.get(triggered_reactions_.front().uid).fqn;
    std::string what{};
    try {
      throw;
    } catch (const std::exception& error) {
      what = ": ";
      what += error.what();
    } catch (...) {
    }
    abort_execution_ = true;
    trigger_shutdown();
    if (active_exception_ == nullptr) {
      active_exception_ = std::current_exception();
      util::log::error() << "Exception caught during execution of reaction " << fqn << what << ". Shutting down.";
    } else {
      util::log::error() << "Dropping exception from reaction " << fqn
                         << " because an earlier exception is already recorded" << what;
    }
  }
}

} // namespace xronos::runtime::default_::detail

// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/detail/scheduler.hh"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <map>
#include <mutex>
#include <ranges>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/abi/value.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/logical_time/tag.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"
#include "xronos/value/boxing.hh"

namespace xronos::runtime::default_::detail {

namespace {

// Backs the ABI's sink-based serializer with an owned buffer the branches
// then deserialize from.
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
  {
    std::lock_guard<std::mutex> lock(mutex_);
    event_queue_[tag].emplace_back(std::move(value), element_uid);
  }
  cv_.notify_one();
}

void EventQueue::trigger_external_event(std::uint64_t element_uid, abi::AnyValue&& value) {
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

    event_queue_[tag].emplace_back(std::move(value), element_uid);
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
    auto& properties = core::get_properties<core::ReactionTag>(
        reactor_model.element_registry.get(runtime_model.ordered_reaction_uids[index]));
    // Guaranteed by validator::check_reaction_handlers on validated models.
    util::assert_(properties.handler != nullptr);
    auto res = reactions_.try_emplace(runtime_model.ordered_reaction_uids[index], index,
                                      runtime_model.ordered_reaction_uids[index], *properties.handler);
    util::assert_(res.second);
  }

  if (!reactor_model.element_registry.elements_of_type<core::PhysicalEventTag>().empty()) {
    event_queue_.set_has_external_triggers();
    has_external_triggers_ = true;
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
  auto& event_source = event_sources_[element_uid];

  event_source.activate(std::move(value));
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

void Scheduler::set_port(std::uint64_t port_uid, abi::AnyValue&& value) {
  util::log::debug() << "Setting port " << reactor_model_->element_registry.get(port_uid).fqn;
  util::assert_(event_sources_.contains(port_uid));

  // Delivery trees are keyed by origin. A port in the middle of a connection
  // chain misses the map: its downstream ports are leaves of the origin's
  // tree and receive their events when the origin is set.
  auto tree_it = runtime_model_->delivery_trees.find(port_uid);
  if (tree_it == runtime_model_->delivery_trees.end()) {
    trigger_event(port_uid, std::move(value));
    return;
  }

  trigger_event(port_uid, abi::AnyValue{value});
  (void)deliver_subtree(tree_it->second, std::move(value));
}

auto Scheduler::deliver_subtree(const ResolvedDeliverySubtree& subtree, abi::AnyValue&& value) noexcept -> bool {
  // Fanning out needs one box per receiver; only the last leaf can steal the
  // incoming value, and only when no serialization still needs it.
  auto deliver_leaf = [this](const core::DeliveryLeaf& leaf, abi::AnyValue&& to_send) {
    if (leaf.delay.has_value()) {
      schedule_event(leaf.port_uid, std::move(to_send), *leaf.delay);
    } else {
      trigger_event(leaf.port_uid, std::move(to_send));
    }
  };
  for (std::size_t index = 0; index < subtree.leaves.size(); index++) {
    if (index + 1 == subtree.leaves.size() && subtree.serializations.empty()) {
      deliver_leaf(subtree.leaves[index], std::move(value));
    } else {
      deliver_leaf(subtree.leaves[index], abi::AnyValue{value});
    }
  }

  for (const auto& serialization : subtree.serializations) {
    // Encode once per boundary port, no matter how many branches leave it.
    std::vector<std::byte> bytes{};
    try {
      VectorByteSink sink{};
      serialization.serializer->serialize(value, sink);
      bytes = std::move(sink).take();
    } catch (...) {
      handle_boundary_error(serialization.serialize_uid, serialization.branches.front().deserialize_uid, true);
      return false;
    }
    for (const auto& branch : serialization.branches) {
      // Each branch delivers the box created by its own deserializer, so the
      // receivers never share the sender's box.
      abi::AnyValue decoded{};
      try {
        decoded = branch.deserializer->deserialize(bytes.data(), bytes.size());
      } catch (...) {
        handle_boundary_error(serialization.serialize_uid, branch.deserialize_uid, false);
        return false;
      }
      if (!deliver_subtree(branch.subtree, std::move(decoded))) {
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
  trigger_shutdown();
  if (active_exception_ == nullptr) {
    active_exception_ = std::make_exception_ptr(std::runtime_error{message});
  } else {
    util::log::warn() << "Dropping exception details as there already is an active exception.";
  }
}

void Scheduler::execute() {
  start_tag_ = logical_time::Tag{std::chrono::system_clock::now(), 0};
  current_tag_ = start_tag_;
  shutdown_tag_ = start_tag_ + execution_properties_.timeout;

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

  // Process all events until we reach the shutdown tag or run out of events
  while (process_next_tag(shutdown_tag_)) {
  }

  util::log::debug() << "Triggering all shutdown events.";

  if (!has_external_triggers_ && !event_queue_.check_has_events_at_tag(shutdown_tag_)) {
    shutdown_tag_ = current_tag_ + core::Duration::zero();
  }

  // Trigger the shutdown events
  for (auto shutdown_uid : runtime_model_->shutdown_trigger_uids) {
    schedule_event(shutdown_uid, xronos::value::make<abi::Void>(), shutdown_tag_);
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

  for (auto& event : event_handle.mapped()) {
    trigger_event(event.element_uid, std::move(event.value));
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
      schedule_event(uid, xronos::value::make<abi::Void>(), it->second.period);
    }
  }
  active_events_.clear();
}

void Scheduler::execute_reaction(abi::ReactionHandler& handler) {
  try {
    handler.invoke();
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

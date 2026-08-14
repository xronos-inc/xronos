// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH
#define XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <map>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "xronos/abi/backend.hh"
#include "xronos/abi/value.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/logical_time/tag.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_::detail {

struct Event {
  abi::AnyValue value;
  std::uint64_t element_uid;
};

struct Reaction {
  std::uint64_t index;
  std::uint64_t uid;
  std::reference_wrapper<abi::ReactionHandler> handler;

  auto operator<=>(const Reaction& other) const noexcept { return index <=> other.index; }
  auto operator==(const Reaction& other) const noexcept { return index == other.index; }
};

class EventSource {
public:
  // The empty value means that no event is present; valueless events carry a
  // boxed `abi::Void`.
  [[nodiscard]] auto get() const noexcept -> const abi::AnyValue& { return value_; }

  void activate(abi::AnyValue&& value) { value_ = std::move(value); }
  void reset() { value_.reset(); }

private:
  abi::AnyValue value_;
};

class EventQueue {
public:
  EventQueue(const ExecutionProperties& properties)
      : fast_mode_(properties.fast_mode) {}

  void schedule_event(std::uint64_t element_uid, abi::AnyValue&& value, const logical_time::Tag& tag);
  void trigger_external_event(std::uint64_t element_uid, abi::AnyValue&& value);
  auto wait_until_next_event(const logical_time::Tag& max_tag)
      -> std::map<logical_time::Tag, std::vector<Event>>::node_type;
  auto check_has_events_at_tag(const logical_time::Tag& tag) const noexcept -> bool;

  void set_has_external_triggers() { has_external_triggers_ = true; }

private:
  std::map<logical_time::Tag, std::vector<Event>> event_queue_{};
  mutable std::mutex mutex_{};
  std::condition_variable cv_{};
  bool has_external_triggers_{false};
  bool fast_mode_;
  core::TimePoint last_external_event_timestamp_{};
};

class Scheduler final {
public:
  Scheduler(const ExecutionProperties& execution_properties)
      : execution_properties_{execution_properties}
      , event_queue_{execution_properties} {}

  void init(const core::ReactorModel& reactor_model, const RuntimeModel& runtime_model);

  auto get_event_source(std::uint64_t element_uid) const -> const EventSource& { return event_sources_[element_uid]; }

  void schedule_event(std::uint64_t element_uid, abi::AnyValue&& value, core::Duration delay);
  void schedule_event(std::uint64_t element_uid, abi::AnyValue&& value, const logical_time::Tag& tag);
  void trigger_external_event(std::uint64_t element_uid, abi::AnyValue&& value) {
    event_queue_.trigger_external_event(element_uid, std::move(value));
  }
  void set_port(std::uint64_t port_uid, abi::AnyValue&& value);
  void trigger_shutdown() { shutdown_tag_ = current_tag_ + core::Duration::zero(); }

  void execute();

  auto get_current_tag() const -> const logical_time::Tag& { return current_tag_; }
  auto get_start_tag() const -> const logical_time::Tag& { return start_tag_; }

private:
  ExecutionProperties execution_properties_;
  EventQueue event_queue_;
  std::deque<Reaction> ready_queue_{};
  logical_time::Tag start_tag_{};
  logical_time::Tag current_tag_{};
  logical_time::Tag shutdown_tag_{};

  std::unordered_map<std::uint64_t, Reaction> reactions_{};
  mutable std::unordered_map<std::uint64_t, EventSource> event_sources_{};
  std::vector<std::uint64_t> active_events_{};

  const RuntimeModel* runtime_model_{nullptr};
  const core::ReactorModel* reactor_model_{nullptr};
  bool has_external_triggers_{false};
  std::exception_ptr active_exception_{nullptr};

  void trigger_event(std::uint64_t element_uid, abi::AnyValue&& value);

  // Delivers `value` to every leaf and boundary crossing of the subtree:
  // leaves first, then one serialization per boundary node and one
  // deserialization per branch, recursing with the decoded box. Returns
  // false when a serializer or deserializer threw. A failure aborts the
  // entire remaining walk: it is logged, stored for the rethrow from
  // execute, and triggers shutdown.
  auto deliver_subtree(const ResolvedDeliverySubtree& subtree, abi::AnyValue&& value) noexcept -> bool;
  void handle_boundary_error(std::uint64_t serialize_uid, std::uint64_t deserialize_uid, bool serializing) noexcept;

  auto process_next_tag(const logical_time::Tag& max_tag) -> bool;
  void execute_all_ready_reactions();
  void reset_all_active_events();
  void execute_reaction(abi::ReactionHandler& handler);
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH

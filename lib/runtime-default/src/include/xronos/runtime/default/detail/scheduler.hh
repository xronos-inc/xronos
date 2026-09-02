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
#include "xronos/abi/types.hh"
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
  auto trigger_external_event(std::uint64_t element_uid, abi::AnyValue&& value) -> abi::TriggerStatus;
  // The next batch of events at one tag, or empty once max_tag is reached.
  // A stop request makes a blocked call return empty without waiting, so the
  // event loop ends promptly.
  auto wait_until_next_event(const logical_time::Tag& max_tag)
      -> std::map<logical_time::Tag, std::vector<Event>>::node_type {
    return next_event(max_tag, true);
  }
  // The events at the shutdown tag. Runs after a stop already ended the
  // event loop, so the wait ignores the still-set stop request; honoring it
  // would return empty before the shutdown reactions run.
  auto wait_for_shutdown_events(const logical_time::Tag& shutdown_bound)
      -> std::map<logical_time::Tag, std::vector<Event>>::node_type {
    return next_event(shutdown_bound, false);
  }
  void discard_events_at_tag(const logical_time::Tag& tag) noexcept;

  // Wakes a blocked wait_until_next_event so it returns empty. Callable from
  // any thread; external events arriving afterwards are rejected as Stopped.
  void request_stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested_ = true;
    }
    cv_.notify_one();
  }
  [[nodiscard]] auto stop_requested() const -> bool {
    std::lock_guard<std::mutex> lock(mutex_);
    return stop_requested_;
  }

  void set_has_external_triggers() { has_external_triggers_ = true; }

  // The scheduler owns the live window, but its state is single-threaded
  // while `trigger_external_event` runs on external threads. The scheduler
  // calls these methods to copy each window transition into fields guarded
  // by `mutex_`, so the status check and the event insertion happen under
  // one lock.
  void mark_started(const logical_time::Tag& shutdown_bound) {
    std::lock_guard<std::mutex> lock(mutex_);
    started_ = true;
    shutdown_bound_ = shutdown_bound;
  }
  void narrow_shutdown_bound(const logical_time::Tag& shutdown_bound) {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_bound_ = shutdown_bound;
  }
  void mark_stopped() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_ = true;
  }

private:
  auto next_event(const logical_time::Tag& max_tag, bool stoppable)
      -> std::map<logical_time::Tag, std::vector<Event>>::node_type;

  std::map<logical_time::Tag, std::vector<Event>> event_queue_{};
  mutable std::mutex mutex_{};
  std::condition_variable cv_{};
  bool has_external_triggers_{false};
  bool fast_mode_;
  core::TimePoint last_external_event_timestamp_{};
  // Live-window state, guarded by `mutex_`.
  bool started_{false};
  bool stopped_{false};
  bool stop_requested_{false};
  logical_time::Tag shutdown_bound_{};
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
  auto trigger_external_event(std::uint64_t element_uid, abi::AnyValue&& value) -> abi::TriggerStatus {
    return event_queue_.trigger_external_event(element_uid, std::move(value));
  }
  void set_port(std::uint64_t port_uid, abi::AnyValue&& value);
  void trigger_shutdown() {
    shutdown_tag_ = current_tag_ + core::Duration::zero();
    event_queue_.narrow_shutdown_bound(shutdown_tag_);
  }
  // Requests a prompt stop from any thread while execute() blocks: the
  // scheduler abandons all remaining events, runs the shutdown reactions one
  // microstep after the last processed tag, and execute() returns normally.
  void request_stop() noexcept { event_queue_.request_stop(); }

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
  // One-shot request to stop executing the current tag's remaining
  // reactions. Either error site sets it; execute_all_ready_reactions
  // consumes it and resets it after its loop, so shutdown reactions still
  // run.
  bool abort_execution_{false};

  void trigger_event(std::uint64_t element_uid, abi::AnyValue&& value);

  // Delivers `value` to every leaf and boundary crossing of the subtree:
  // leaves first, then one serialization per boundary node and one
  // deserialization per branch, recursing with the decoded box. Returns
  // false when a serializer or deserializer threw. A failure aborts the
  // entire remaining walk: it is logged, stored for the rethrow from
  // execute, and triggers shutdown.
  auto deliver_subtree(const ResolvedDeliverySubtree& subtree, abi::AnyValue&& value) noexcept -> bool;
  void handle_boundary_error(std::uint64_t serialize_uid, std::uint64_t deserialize_uid, bool serializing) noexcept;

  // Extracts and processes the next tag's events. Returns false once
  // max_tag is reached or a stop request ends the wait.
  auto process_next_tag(const logical_time::Tag& max_tag) -> bool;
  // Runs the reactions scheduled at the shutdown tag. The wait ignores a
  // stop request: shutdown reactions are the point of a graceful stop.
  void process_shutdown_tag();
  // The shared tail of both: advances the current tag and runs the extracted
  // events' reactions. Returns false on an empty handle.
  auto process_events(std::map<logical_time::Tag, std::vector<Event>>::node_type event_handle) -> bool;
  void execute_all_ready_reactions();
  void reset_all_active_events();
  void execute_reaction(abi::ReactionHandler& handler);
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH

// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH
#define XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH

#include <any>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <map>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/logical_time/tag.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/runtime/interfaces.hh"

namespace xronos::runtime::default_::detail {

struct Event {
  std::any value;
  std::uint64_t element_uid;
};

struct Reaction {
  std::uint64_t index;
  std::uint64_t uid;
  std::reference_wrapper<std::function<void()>> handler;

  auto operator<=>(const Reaction& other) const noexcept { return index <=> other.index; }
  auto operator==(const Reaction& other) const noexcept { return index == other.index; }
};

class EventSource {
public:
  [[nodiscard]] auto get() const noexcept -> std::any { return value_; }
  [[nodiscard]] auto is_present() const noexcept -> bool { return value_.has_value(); }

  void activate(const std::any& value) { value_ = value; }
  void reset() { value_.reset(); }

private:
  std::any value_;
};

class EventQueue {
public:
  EventQueue(const ExecutionProperties& properties)
      : fast_mode_(properties.fast_mode) {}

  void schedule_event(std::uint64_t element_uid, const std::any& value, const logical_time::Tag& tag);
  void trigger_external_event(std::uint64_t element_uid, const std::any& value);
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

  void schedule_event(std::uint64_t element_uid, const std::any& value, core::Duration delay);
  void schedule_event(std::uint64_t element_uid, const std::any& value, const logical_time::Tag& tag);
  void trigger_external_event(std::uint64_t element_uid, const std::any& value) {
    event_queue_.trigger_external_event(element_uid, value);
  }
  void set_port(std::uint64_t port_uid, const std::any& value);
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

  void trigger_event(std::uint64_t element_uid, const std::any& value);

  auto process_next_tag(const logical_time::Tag& max_tag) -> bool;
  void execute_all_ready_reactions();
  void reset_all_active_events();
  void execute_reaction(const std::function<void()>& handler);
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_SCHEDULER_HH

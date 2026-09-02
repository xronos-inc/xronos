// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PHYSICAL_EVENT_HH
#define XRONOS_SDK_PHYSICAL_EVENT_HH

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "xronos/abi/backend.hh"
#include "xronos/abi/types.hh"
#include "xronos/abi/value.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/value.hh"
#include "xronos/value/boxing.hh"

namespace xronos::sdk {

/**
 * The outcome of a @ref PhysicalEvent trigger attempt.
 *
 * - `TriggerStatus::Accepted`: the runtime accepted the event and it is queued for processing.
 * - `TriggerStatus::NotStarted`: the program has not started yet and the event was dropped.
 * - `TriggerStatus::Stopped`: the program has stopped and the event was dropped.
 * - `TriggerStatus::Unknown`: the implementation reported a status this SDK version does not recognize; the event
 *   was dropped.
 */
enum class TriggerStatus : std::uint8_t {
  Accepted = 0,
  NotStarted = 1,
  Stopped = 2,
  Unknown = 3,
};

namespace detail {

inline auto register_physical_event(std::string_view name, const ReactorContext& context) -> std::uint64_t {
  return register_with_location(context, [&]() {
    return get_backend(context).register_physical_event(std::string{name}, ContextAccess::get_parent_uid(context));
  });
}

class UntypedPhysicalEvent {
public:
  UntypedPhysicalEvent(std::uint64_t uid, const ReactorContext& context)
      : uid_{uid}
      , program_context_{*ContextAccess::get_program_context(context)} {}

  // Routes every attempt through the backend, which owns the program's
  // live/dead state; holding no pointer into program-owned memory is what
  // keeps concurrent triggers safe across the end of a run (see
  // abi::RuntimeBackend::trigger_physical_event).
  [[nodiscard]] auto trigger(abi::AnyValue&& value) -> TriggerStatus {
    switch (program_context_.get().runtime_backend().trigger_physical_event(uid_, std::move(value))) {
    case abi::TriggerStatus::NotStarted:
      return TriggerStatus::NotStarted;
    case abi::TriggerStatus::Accepted:
      return TriggerStatus::Accepted;
    case abi::TriggerStatus::Stopped:
      return TriggerStatus::Stopped;
    case abi::TriggerStatus::UnknownPhysicalEvent:
      // The uid comes from this element's registration, so the running
      // program resolves it; this status means the SDK or the
      // implementation broke that invariant. The `void` overload of
      // `PhysicalEvent::trigger` is noexcept, so there the throw
      // terminates.
      throw std::logic_error{"The implementation resolved no physical event for uid " + std::to_string(uid_) +
                             ", which came from registration."};
    default:
      // An unknown status appended in a newer ABI minor.
      return TriggerStatus::Unknown;
    }
  }

private:
  std::uint64_t uid_;
  std::reference_wrapper<const detail::ProgramContext> program_context_;
};

} // namespace detail

/**
 * A reactor element for receiving events from external sources.
 *
 * Physical events may be used to trigger reactions from a context outside of
 * the scope of the reactor program. These could be external event handlers that
 * respond to sensor inputs.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger" allowing the
 * reaction handler to read the associated value.
 *
 * @ref trigger may be called from any thread, and is safe while this element
 * and its @ref Environment are alive. This includes calls concurrent with
 * program startup, with shutdown, and with @ref Environment::execute
 * returning. Once the program has stopped, every attempt returns
 * `TriggerStatus::Stopped`; sensor threads should treat that status as the
 * signal to exit.
 *
 * @tparam T The type of values carried by emitted events.
 */
template <class T> class PhysicalEvent final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the physical event.
   * @param context The containing reactor's context.
   */
  PhysicalEvent(std::string_view name, const ReactorContext& context)
      : Element{detail::register_physical_event(name, context), name, context}
      , impl_{uid(), context} {}

  /**
   * Emit a new event with an associated value and trigger reactions.
   *
   * The event will be assigned a timestamp equal to the current wall-clock
   * time.
   *
   * May be called from any thread while this element and its @ref
   * Environment are alive (see @ref PhysicalEvent). Events emitted while
   * the program is not running are dropped, and the status reports why.
   *
   * @param value The value to be associated with the emitted event.
   * @return The outcome of the attempt. `TriggerStatus::Accepted` on success.
   */
  template <class U>
  [[nodiscard]] auto trigger(const U& value) -> TriggerStatus
    requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
  {
    return impl_.trigger(xronos::value::make<T>(value));
  }
  /**
   * @overload
   * @details Move constructs the value using the given rvalue reference.
   */
  template <class U>
  [[nodiscard]] auto trigger(U&& value) -> TriggerStatus
    requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
  {
    return impl_.trigger(xronos::value::make<T>(std::forward<U>(value)));
  }
  /**
   * @overload
   * @details Emits the given value without copying it.
   * @throws std::invalid_argument If the value is empty.
   */
  [[nodiscard]] auto trigger(const Value<T>& value) -> TriggerStatus
    requires(!std::is_same_v<T, void>)
  {
    if (value == nullptr) {
      throw std::invalid_argument{"An empty Value may not be sent through a physical event."};
    }
    return impl_.trigger(abi::AnyValue{detail::ValueAccess::unwrap(value)});
  }
  /**
   * @overload
   * @details Emits the viewed value without copying it.
   * @throws std::invalid_argument If the view is absent.
   */
  [[nodiscard]] auto trigger(const ValueView<T>& view) -> TriggerStatus
    requires(!std::is_same_v<T, void>)
  {
    // Check the view's presence, not the unwrapped pointer: a view obtained
    // from an absent trigger refers to an *empty* value, so the pointer is
    // non-null but nothing must be emitted.
    if (view == nullptr) {
      throw std::invalid_argument{"An absent ValueView may not be sent through a physical event."};
    }
    return impl_.trigger(abi::AnyValue{*detail::ValueAccess::unwrap(view)});
  }

  /**
   * @overload
   *
   * @details Emits an event without an associated value. This is only available if
   * `T` is `void`.
   */
  [[nodiscard]] auto trigger() noexcept -> TriggerStatus
    requires(std::is_same_v<T, void>)
  {
    return impl_.trigger(xronos::value::make<abi::Void>());
  }

  // Disambiguate trigger(0) by explicitly deleting trigger(nullptr_t)
  template <typename V>
  void trigger(V)
    requires(std::is_same_v<V, std::nullptr_t>)
  = delete;

private:
  detail::UntypedPhysicalEvent impl_;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PHYSICAL_EVENT_HH

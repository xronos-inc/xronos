// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PHYSICAL_EVENT_HH
#define XRONOS_SDK_PHYSICAL_EVENT_HH

#include <any>
#include <string_view>
#include <variant>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

namespace detail::runtime_physical_event {

// Helper functions for accessing the underlying runtime code using pImpl.
void trigger(Element& event, const std::any& value);
[[nodiscard]] auto is_present(const Element& event) noexcept -> bool;
[[nodiscard]] auto get(const Element& event) noexcept -> const std::any&;
void register_as_trigger_of(const Element& event, runtime::Reaction& reaction) noexcept;
auto make_instance(std::string_view name, ReactorContext context) -> RuntimeElementPtr;

} // namespace detail::runtime_physical_event

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
 * @tparam T The type of values carried by emitted events.
 */
template <class T> class PhysicalEvent final : public Element, public EventSource<T> {
public:
  /**
   * Constructor.
   *
   * @param name The name of the physical event.
   * @param context The containing reactor's context.
   */
  PhysicalEvent(std::string_view name, ReactorContext context)
      : Element{detail::runtime_physical_event::make_instance(name, context), context} {}

  /**
   * Emit a new event with an associated value and trigger reactions.
   *
   * The event will be assigned a timestamp equal to the current wall-clock
   * time.
   *
   * @param value The value to be associated with the emitted event.
   */
  void trigger(const ImmutableValuePtr<T>& value) noexcept { detail::runtime_physical_event::trigger(*this, value); }
  /** @overload */
  void trigger(MutableValuePtr<T>&& value_ptr) { trigger(ImmutableValuePtr<T>{std::move(value_ptr)}); }
  /**
   * @overload
   * @details Copy constructs the value using the given lvalue reference.
   */
  void trigger(const T& value) { trigger(make_immutable_value<T>(value)); }
  /**
   * @overload
   * @details Move constructs the value using the given rvalue reference.
   */
  void trigger(T&& value) { trigger(make_immutable_value<T>(std::move(value))); }

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::runtime_physical_event::is_present(*this);
  }
  [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T> final {
    if (!is_present()) {
      return ImmutableValuePtr<T>{nullptr};
    }
    return std::any_cast<ImmutableValuePtr<T>>(detail::runtime_physical_event::get(*this));
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    detail::runtime_physical_event::register_as_trigger_of(*this, reaction);
  }
};

/**
 * A reactor element for receiving events from external sources.
 *
 * Physical events may be used to trigger reactions from a context outside of
 * the scope of the reactor program. These could be external event handlers that
 * respond to sensor inputs.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger".
 *
 * This is a template specialization of EventSource for events without an
 * associated value.
 */
template <> class PhysicalEvent<void> final : public Element, public EventSource<void> {
public:
  /** @copydoc PhysicalEvent::PhysicalEvent */
  PhysicalEvent(std::string_view name, ReactorContext context)
      : Element{detail::runtime_physical_event::make_instance(name, context), context} {}

  /**
   * Emit a new event without an associated value and trigger reactions.
   *
   * The event will be assigned a timestamp equal to the current wall-clock
   * time.
   */
  void trigger() noexcept { detail::runtime_physical_event::trigger(*this, std::monostate{}); }

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::runtime_physical_event::is_present(*this);
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    detail::runtime_physical_event::register_as_trigger_of(*this, reaction);
  }
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PHYSICAL_EVENT_HH

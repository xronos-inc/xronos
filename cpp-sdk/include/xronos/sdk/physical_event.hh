// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PHYSICAL_EVENT_HH
#define XRONOS_SDK_PHYSICAL_EVENT_HH

#include <any>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>
#include <variant>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

namespace detail {

auto register_physical_event(std::string_view name, const ReactorContext& context) -> const core::Element&;

class PhysicalEventImpl {
public:
  PhysicalEventImpl(std::uint64_t uid, const ReactorContext& context);
  void trigger(const std::any& value);

private:
  std::uint64_t uid_;
  std::reference_wrapper<const detail::ProgramContext> program_context_;

  runtime::ExternalTrigger* impl_{nullptr};
  auto get_impl() noexcept -> runtime::ExternalTrigger*;
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
      : Element{detail::register_physical_event(name, context), context}
      , impl_{uid(), context} {}

  /**
   * Emit a new event with an associated value and trigger reactions.
   *
   * The event will be assigned a timestamp equal to the current wall-clock
   * time.
   *
   * @param value The value to be associated with the emitted event.
   */
  void trigger(const ImmutableValuePtr<T>& value) noexcept
    requires(!std::is_same_v<T, void>)
  {
    impl_.trigger(value);
  }
  /** @overload */
  void trigger(MutableValuePtr<T>&& value_ptr)
    requires(!std::is_same_v<T, void>)
  {
    trigger(ImmutableValuePtr<T>{std::move(value_ptr)});
  }
  /**
   * @overload
   * @details Copy constructs the value using the given lvalue reference.
   */
  template <class U>
  void trigger(const U& value)
    requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
  {
    trigger(make_immutable_value<T>(value));
  }
  /**
   * @overload
   * @details Move constructs the value using the given rvalue reference.
   */
  template <class U>
  void trigger(U&& value)
    requires(!std::is_same_v<U, void> && std::is_same_v<T, U>)
  {
    trigger(make_immutable_value<T>(std::forward<U>(value)));
  }

  /**
   * @overload
   *
   * @details Emits an event without an associated value. This is only available if
   * `T` is `void`.
   */
  void trigger() noexcept
    requires(std::is_same_v<T, void>)
  {
    impl_.trigger(std::monostate{});
  }

  // Disambiguate trigger(0) by explicitly deleting trigger(nullptr_t)
  template <typename V>
  void trigger(V)
    requires(std::is_same_v<V, std::nullptr_t>)
  = delete;

private:
  detail::PhysicalEventImpl impl_;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PHYSICAL_EVENT_HH

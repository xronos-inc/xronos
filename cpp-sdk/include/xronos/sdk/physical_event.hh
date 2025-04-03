// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `PhysicalEvent` class.
 */

#ifndef XRONOS_SDK_PHYSICAL_EVENT_HH
#define XRONOS_SDK_PHYSICAL_EVENT_HH

#include <memory>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/value_ptr.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::sdk {
/**
 * @brief An element for scheduling new events from external contexts.
 *
 * @details Physical events may be used to schedule new events from an external
 * context outside of the scope of the reactor program. These could be external
 * event handlers that respond to sensor inputs. Physical events do not provide
 * direct access to their values. A reaction may declare a
 * `BaseReaction::Trigger` or `BaseReaction::Source` to access the value
 * associated with an active event.
 *
 * @tparam T The type of values carried by the event.
 */
template <class T> class PhysicalEvent final : public Element, public EventSource<T> {
public:
  /**
   * @brief Construct a new `PhysicalEvent`.
   *
   * @param name The name of the `PhysicalEvent`.
   * @param context The context object obtained from the `PhysicalEvent`'s
   * containing reactor.
   */
  PhysicalEvent(std::string_view name, ReactorContext context)
      : Element{std::make_unique<runtime::PhysicalAction<T>>(name, detail::get_reactor_instance(context)), context} {}

  /**
   * @brief Create a new event.
   *
   * @details This will automatically assign a timestamp to the newly created
   * event. This timestamp will be derived from the system clock and will align
   * with real time on a best-effort basis.
   * @param value A value to be associated with the newly created event.
   */
  void trigger(const ImmutableValuePtr<T>& value) noexcept {
    detail::get_runtime_instance<runtime::PhysicalAction<T>>(*this).schedule(value);
  }

  /**
   * @overload
   */
  void trigger(MutableValuePtr<T>&& value_ptr) { trigger(ImmutableValuePtr<T>{std::move(value_ptr)}); }

  /**
   * @overload
   */
  void trigger(const T& value) { trigger(make_immutable_value<T>(value)); }

  /**
   * @overload
   */
  void trigger(T&& value) { trigger(make_immutable_value<T>(std::move(value))); }

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::get_runtime_instance<runtime::PhysicalAction<T>>(*this).is_present();
  }
  [[nodiscard]] auto get() const noexcept -> const ImmutableValuePtr<T>& final {
    return detail::get_runtime_instance<runtime::PhysicalAction<T>>(*this).get();
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_trigger(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }

  void register_as_source_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_dependency(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }
};
/**
 * @brief An element for scheduling new events from external contexts.
 *
 * @details Physical events may be used to schedule new events from an external
 * context outside of the scope of the reactor program. These could be external
 * event handlers that respond to sensor inputs. Physical events do not provide
 * direct access to their values. A reaction may declare a
 * `BaseReaction::Trigger` or `BaseReaction::Source` to access the value
 * associated with an active event.
 *
 * @details This specialization is used for physical events that do not convey
 * any values.
 */
template <> class PhysicalEvent<void> final : public Element, public EventSource<void> {
public:
  /**
   * @brief Construct a new `PhysicalEvent`.
   *
   * @param name The name of the `PhysicalEvent`.
   * @param context The context object obtained from the `PhysicalEvent`'s
   * containing reactor.
   */
  PhysicalEvent(std::string_view name, ReactorContext context)
      : Element{std::make_unique<runtime::PhysicalAction<void>>(name, detail::get_reactor_instance(context)), context} {
  }
  /**
   * @brief Create a new (valueless) event.
   *
   * @details This will automatically assign a timestamp to the newly created event.
   */
  void trigger() noexcept { detail::get_runtime_instance<runtime::PhysicalAction<void>>(*this).schedule(); }

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::get_runtime_instance<runtime::PhysicalAction<void>>(*this).is_present();
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_trigger(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }

  void register_as_source_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_dependency(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PHYSICAL_EVENT_HH

// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `ProgrammableTimer` class.
 */
#ifndef XRONOS_SDK_PROGRAMMABLE_TIMER_HH
#define XRONOS_SDK_PROGRAMMABLE_TIMER_HH

#include <memory>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::sdk {
/**
 * @brief A reactor element for scheduling new events.
 *
 * @details Programmable timers may be used by reactions to schedule new events in
 * the future. Events are not scheduled or read directly. Instead, reactions
 * may declare a `BaseReaction::ProgrammableTimerEffect` to schedule new events, or
 * a `BaseReaction::Trigger`
 * or `BaseReaction::Source` to access the value associated with an active event.
 *
 * @tparam T The type of values carried by the programmable timer.
 */
template <class T> class ProgrammableTimer final : public Element, public EventSource<T> {
public:
  /**
   * @brief Construct a new `ProgrammableTimer`.
   *
   * @param name The name of the `ProgrammableTimer`.
   * @param context The context object obtained from the `ProgrammableTimer`'s
   * containing reactor.
   */
  ProgrammableTimer(std::string_view name, ReactorContext context)
      : Element{std::make_unique<runtime::LogicalAction<T>>(name, detail::get_reactor_instance(context)), context} {}

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::get_runtime_instance<runtime::LogicalAction<T>>(*this).is_present();
  }
  [[nodiscard]] auto get() const noexcept -> const ImmutableValuePtr<T>& final {
    return detail::get_runtime_instance<runtime::LogicalAction<T>>(*this).get();
  }

  void schedule(const ImmutableValuePtr<T>& value, Duration delay = Duration::zero()) noexcept {
    detail::get_runtime_instance<runtime::LogicalAction<T>>(*this).schedule(value, delay);
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_trigger(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }

  void register_as_source_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_dependency(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }

  friend BaseReaction;
};
/**
 * @brief An element for scheduling new events.
 *
 * @details Programmable timers may be used by reactions to schedule new events in
 * the future. Events are not scheduled or read directly. Instead, reactions
 * may declare a `BaseReaction::ProgrammableTimerEffect` to schedule new events, or
 * a `BaseReaction::Trigger` or `BaseReaction::Source` to access the value
 * associated with an active event.
 *
 * @details This specialization is used for programmable timers that do not convey
 * any values.
 */
template <> class ProgrammableTimer<void> final : public Element, public EventSource<void> {
public:
  /**
   * @brief Construct a new `ProgrammableTimer`.
   *
   * @param name The name of the `ProgrammableTimer`.
   * @param context The context object obtained from the `ProgrammableTimer`'s
   * containing reactor.
   */
  ProgrammableTimer(std::string_view name, ReactorContext context)
      : Element{std::make_unique<runtime::LogicalAction<void>>(name, detail::get_reactor_instance(context)), context} {}

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::get_runtime_instance<runtime::LogicalAction<void>>(*this).is_present();
  }

  void schedule(Duration delay = Duration::zero()) noexcept {
    detail::get_runtime_instance<runtime::LogicalAction<void>>(*this).schedule(delay);
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_trigger(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }

  void register_as_source_of(runtime::Reaction& reaction) const noexcept final {
    reaction.declare_dependency(&detail::get_runtime_instance<runtime::BaseAction>(*this));
  }

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PROGRAMMABLE_TIMER_HH

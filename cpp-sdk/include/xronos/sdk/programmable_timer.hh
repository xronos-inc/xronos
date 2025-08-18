// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PROGRAMMABLE_TIMER_HH
#define XRONOS_SDK_PROGRAMMABLE_TIMER_HH

#include <any>
#include <string_view>
#include <variant>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"
#include "xronos/sdk/value_ptr.hh"

namespace xronos::sdk {

namespace detail::runtime_programmable_timer {

// Helper functions for accessing the underlying runtime code using pImpl.
void schedule(Element& timer, const std::any& value, Duration delay = Duration::zero()) noexcept;
[[nodiscard]] auto is_present(const Element& timer) noexcept -> bool;
[[nodiscard]] auto get(const Element& timer) noexcept -> const std::any&;
void register_as_trigger_of(const Element& timer, runtime::Reaction& reaction) noexcept;
auto make_instance(std::string_view name, ReactorContext context) -> RuntimeElementPtr;
void register_as_effect_of(const Element& timer, runtime::Reaction& reaction) noexcept;

} // namespace detail::runtime_programmable_timer

/**
 * A reactor element for scheduling new events.
 *
 * Programmable timers may be used by reactions to schedule new events that will
 * be emitted in the future. They can be used both as a reaction @ref
 * BaseReaction::Trigger "trigger" and an reaction @ref
 * BaseReaction::ProgrammableTimerEffect "effect".
 *
 * @tparam T The value type associated with events emitted by the programmable timer.
 */
template <class T> class ProgrammableTimer final : public Element, public EventSource<T> {
public:
  /**
   * Constructor.
   *
   * @param name The name of the `ProgrammableTimer`.
   * @param context The containing reactor's context.
   */
  ProgrammableTimer(std::string_view name, ReactorContext context)
      : Element{detail::runtime_programmable_timer::make_instance(name, context), context} {}

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::runtime_programmable_timer::is_present(*this);
  }

  [[nodiscard]] auto get() const noexcept -> ImmutableValuePtr<T> final {
    if (!is_present()) {
      return ImmutableValuePtr<T>{nullptr};
    }
    return std::any_cast<ImmutableValuePtr<T>>(detail::runtime_programmable_timer::get(*this));
  }

  void schedule(const ImmutableValuePtr<T>& value, Duration delay = Duration::zero()) noexcept {
    detail::runtime_programmable_timer::schedule(*this, value, delay);
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    detail::runtime_programmable_timer::register_as_trigger_of(*this, reaction);
  }

  friend BaseReaction;
};

/**
 * A reactor element for scheduling new events.
 *
 * Programmable timers may be used by reactions to schedule new events that will
 * be emitted in the future. They can be used both as an reaction @ref
 * BaseReaction::Trigger "trigger" and an reaction @ref
 * BaseReaction::ProgrammableTimerEffect "effect".
 *
 * This is a template specialization of ProgrammableTimer for scheduling and
 * emitting events that do not have an associated value.
 */
template <> class ProgrammableTimer<void> final : public Element, public EventSource<void> {
public:
  /** @copydoc ProgrammableTimer::ProgrammableTimer */
  ProgrammableTimer(std::string_view name, ReactorContext context)
      : Element{detail::runtime_programmable_timer::make_instance(name, context), context} {}

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final {
    return detail::runtime_programmable_timer::is_present(*this);
  }

  void schedule(Duration delay = Duration::zero()) noexcept {
    detail::runtime_programmable_timer::schedule(*this, std::monostate{}, delay);
  }

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final {
    detail::runtime_programmable_timer::register_as_trigger_of(*this, reaction);
  }

  friend BaseReaction;
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PROGRAMMABLE_TIMER_HH

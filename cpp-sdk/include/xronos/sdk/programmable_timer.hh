// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PROGRAMMABLE_TIMER_HH
#define XRONOS_SDK_PROGRAMMABLE_TIMER_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/context_access.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk {

namespace detail {

inline auto register_programmable_timer(std::string_view name, const ReactorContext& context) -> std::uint64_t {
  return register_with_location(context, [&]() -> std::uint64_t {
    return get_backend(context).register_programmable_timer(std::string{name}, ContextAccess::get_parent_uid(context));
  });
}

} // namespace detail

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
template <class T> class ProgrammableTimer final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the `ProgrammableTimer`.
   * @param context The containing reactor's context.
   */
  ProgrammableTimer(std::string_view name, const ReactorContext& context)
      : Element{detail::register_programmable_timer(name, context), name, context} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PROGRAMMABLE_TIMER_HH

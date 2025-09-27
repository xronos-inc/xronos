// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PROGRAMMABLE_TIMER_HH
#define XRONOS_SDK_PROGRAMMABLE_TIMER_HH

#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk {

namespace detail {

auto register_programmable_timer(std::string_view name, const ReactorContext& context) -> const core::Element&;

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
      : Element{detail::register_programmable_timer(name, context), context} {}
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PROGRAMMABLE_TIMER_HH

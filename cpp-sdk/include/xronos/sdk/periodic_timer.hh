// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PERIODIC_TIMER_HH
#define XRONOS_SDK_PERIODIC_TIMER_HH

#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

namespace detail {

void set_timer_period(PeriodicTimer& timer, Duration period);
void set_timer_offset(PeriodicTimer& timer, Duration offset);

} // namespace detail

/**
 * A reactor element that emits events in regular intervals.
 *
 * Can be used as a reaction @ref BaseReaction::Trigger "trigger".
 */
class PeriodicTimer final : public Element {
public:
  /**
   * Constructor.
   *
   * @param name The name of the periodic timer.
   * @param context The containing reactor's context.
   * @param period The delay in between two events emitted by the timer.
   * @param offset The delay between the startup event and the first event
   * emitted by the timer.
   */
  PeriodicTimer(std::string_view name, ReactorContext context, Duration period, Duration offset = Duration::zero());

  /**
   * Get the delay in between two events emitted by the timer.
   *
   * @return The timer's period.
   */
  [[nodiscard]] auto period() const noexcept -> const Duration&;
  /**
   * Get the timer's offset.
   *
   * The offset denotes the delay between the startup event and the first event
   * emitted by the timer.
   *
   * @return The timer's offset.
   */
  [[nodiscard]] auto offset() const noexcept -> const Duration&;

private:
  friend void detail::set_timer_period(PeriodicTimer& timer, Duration period);
  friend void detail::set_timer_offset(PeriodicTimer& timer, Duration offset);
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PERIODIC_TIMER_HH

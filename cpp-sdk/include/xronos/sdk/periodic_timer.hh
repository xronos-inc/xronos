// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file
 *
 * @brief Definition of the `PeriodicTimer` class.
 */

#ifndef XRONOS_SDK_PERIODIC_TIMER_HH
#define XRONOS_SDK_PERIODIC_TIMER_HH

#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/event_source.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

namespace detail {

void set_timer_period(PeriodicTimer& timer, Duration period);
void set_timer_offset(PeriodicTimer& timer, Duration offset);

} // namespace detail

/**
 * @brief An event source that emits events in regular intervals.
 */
class PeriodicTimer final : public EventSource<void> {
public:
  /**
   * @brief Construct a new `PeriodicTimer` object.
   *
   * @param name The name of the timer event.
   * @param context The current reactor's initialization context, which can be
   * obtained using the `Reactor::context` method.
   * @param period The delay in between two events emitted by the timer. See
   * `Timer::period` for details.
   * @param offset The delay between `Reactor::startup` and the first event
   * emitted by the timer.
   */
  PeriodicTimer(std::string_view name, ReactorContext context, Duration period, Duration offset = Duration::zero());

  /**
   * @brief The delay in between two events emitted by the timer.
   *
   * @details If `period` is 0 (the default), then the timer will trigger only
   * once with a delay of `offset` after `Reactor::startup` triggers. If
   * `offset` is also set to 0, the timer triggers simultaneously to
   * `Reactor::startup`.
   * @return const Duration& The delay in between two events emitted by the
   * timer.
   */
  [[nodiscard]] auto period() const noexcept -> const Duration&;
  /**
   * @brief The delay between `Reactor::startup` and the first event emitted by
   * the timer.
   *
   * @return const Duration& The delay between `Reactor::startup` and the first
   * event emitted by the timer.
   */
  [[nodiscard]] auto offset() const noexcept -> const Duration&;

private:
  [[nodiscard]] auto is_present() const noexcept -> bool final;

  void register_as_trigger_of(runtime::Reaction& reaction) const noexcept final;

  friend void detail::set_timer_period(PeriodicTimer& timer, Duration period);
  friend void detail::set_timer_offset(PeriodicTimer& timer, Duration offset);
};

} // namespace xronos::sdk

#endif // XRONOS_SDK_PERIODIC_TIMER_HH

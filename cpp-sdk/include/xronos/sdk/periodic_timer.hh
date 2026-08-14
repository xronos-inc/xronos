// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

/** @file */

#ifndef XRONOS_SDK_PERIODIC_TIMER_HH
#define XRONOS_SDK_PERIODIC_TIMER_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/sdk/context.hh"
#include "xronos/sdk/detail/element.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

namespace detail {

void set_timer_period(PeriodicTimer& timer, Duration period);
void set_timer_offset(PeriodicTimer& timer, Duration offset);

inline auto register_periodic_timer(std::string_view name, const ReactorContext& context, Duration offset,
                                    Duration period) -> std::uint64_t {
  return register_with_location(context, [&]() -> std::uint64_t {
    return get_backend(context).register_periodic_timer(std::string{name}, ContextAccess::get_parent_uid(context),
                                                        offset, period);
  });
}

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
  PeriodicTimer(std::string_view name, ReactorContext context, Duration period, Duration offset = Duration::zero())
      : Element{detail::register_periodic_timer(name, context, offset, period), name, context}
      , period_{period}
      , offset_{offset} {}

  /**
   * Get the delay in between two events emitted by the timer.
   *
   * @return The timer's period.
   */
  [[nodiscard]] auto period() const noexcept -> const Duration& { return period_; }
  /**
   * Get the timer's offset.
   *
   * The offset denotes the delay between the startup event and the first event
   * emitted by the timer.
   *
   * @return The timer's offset.
   */
  [[nodiscard]] auto offset() const noexcept -> const Duration& { return offset_; }

private:
  Duration period_;
  Duration offset_;

  friend void detail::set_timer_period(PeriodicTimer& timer, Duration period);
  friend void detail::set_timer_offset(PeriodicTimer& timer, Duration offset);
};

namespace detail {

// Defined below the class: these touch the timer's private state through
// their friend declarations.
inline void set_timer_period(PeriodicTimer& timer, Duration period) {
  timer.program_context()->backend().set_periodic_timer_period(timer.uid(), period);
  timer.period_ = period;
}

inline void set_timer_offset(PeriodicTimer& timer, Duration offset) {
  timer.program_context()->backend().set_periodic_timer_offset(timer.uid(), offset);
  timer.offset_ = offset;
}

} // namespace detail

} // namespace xronos::sdk

#endif // XRONOS_SDK_PERIODIC_TIMER_HH

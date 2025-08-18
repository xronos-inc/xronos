// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/periodic_timer.hh"

#include <string_view>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

PeriodicTimer::PeriodicTimer(std::string_view name, ReactorContext context, Duration period, Duration offset)
    : Element{detail::make_runtime_element_pointer<runtime::Timer>(name, detail::get_reactor_instance(context), period,
                                                                   offset),
              context} {}

[[nodiscard]] auto PeriodicTimer::period() const noexcept -> const Duration& {
  return detail::get_runtime_instance<runtime::Timer>(*this).period();
}

[[nodiscard]] auto PeriodicTimer::offset() const noexcept -> const Duration& {
  return detail::get_runtime_instance<runtime::Timer>(*this).offset();
}

[[nodiscard]] auto PeriodicTimer::is_present() const noexcept -> bool {
  return detail::get_runtime_instance<runtime::Timer>(*this).is_present();
}

void PeriodicTimer::register_as_trigger_of(runtime::Reaction& reaction) const noexcept {
  reaction.declare_trigger(&detail::get_runtime_instance<runtime::Timer>(*this));
}

namespace detail {

void set_timer_period(PeriodicTimer& timer, Duration period) {
  get_runtime_instance<runtime::Timer>(timer).set_period(period);
}

void set_timer_offset(PeriodicTimer& timer, Duration offset) {
  get_runtime_instance<runtime::Timer>(timer).set_offset(offset);
}

} // namespace detail

} // namespace xronos::sdk

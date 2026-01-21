// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/periodic_timer.hh"

#include <memory>
#include <string_view>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "impl/xronos/sdk/detail/element.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk {

using CA = detail::ContextAccess;

PeriodicTimer::PeriodicTimer(std::string_view name, ReactorContext context, Duration period, Duration offset)
    : Element{detail::register_element(name,
                                       core::PeriodicTimerTag{std::make_unique<core::PeriodicTimerProperties>(
                                           core::PeriodicTimerProperties{.offset = offset, .period = period})},
                                       context),
              context} {}

[[nodiscard]] auto PeriodicTimer::period() const noexcept -> const Duration& {
  return core::get_properties<core::PeriodicTimerTag>(core_element()).period;
}

[[nodiscard]] auto PeriodicTimer::offset() const noexcept -> const Duration& {
  return core::get_properties<core::PeriodicTimerTag>(core_element()).offset;
}

namespace detail {

void set_timer_period(PeriodicTimer& timer, Duration period) {
  core::get_properties<core::PeriodicTimerTag>(timer.core_element()).period = period;
}

void set_timer_offset(PeriodicTimer& timer, Duration offset) {
  core::get_properties<core::PeriodicTimerTag>(timer.core_element()).offset = offset;
}

} // namespace detail

} // namespace xronos::sdk

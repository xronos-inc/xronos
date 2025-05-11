// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <stdexcept>

#include "xronos/sdk/periodic_timer.hh"
#include "xronos/sdk/time.hh"

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"

namespace xronos::sdk {

PeriodicTimer::PeriodicTimer(std::string_view name, ReactorContext context, Duration period, Duration offset)
    : Element{std::make_unique<runtime::Timer>(name, detail::get_reactor_instance(context), period, offset), context} {
  if (period <= Duration::zero()) {
    throw std::runtime_error("Timer period must be greater than zero.");
  }
  if (offset < Duration::zero()) {
    throw std::runtime_error("Timer offset must be greater than or equal to zero.");
  }
}

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

} // namespace xronos::sdk

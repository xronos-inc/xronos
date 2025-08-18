// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/programmable_timer.hh"

#include <any>
#include <string_view>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk::detail::runtime_programmable_timer {

auto make_instance(std::string_view name, ReactorContext context) -> RuntimeElementPtr {
  return detail::make_runtime_element_pointer<runtime::LogicalAction>(name, detail::get_reactor_instance(context));
}

void schedule(Element& timer, const std::any& value, Duration delay) noexcept {
  detail::get_runtime_instance<runtime::LogicalAction>(timer).schedule(value, delay);
}

auto is_present(const Element& timer) noexcept -> bool {
  return detail::get_runtime_instance<runtime::LogicalAction>(timer).is_present();
}

auto get(const Element& timer) noexcept -> const std::any& {
  return detail::get_runtime_instance<runtime::LogicalAction>(timer).get();
}

void register_as_trigger_of(const Element& timer, runtime::Reaction& reaction) noexcept {
  reaction.declare_trigger(&detail::get_runtime_instance<runtime::LogicalAction>(timer));
}

void register_as_effect_of(const Element& timer, runtime::Reaction& reaction) noexcept {

  reaction.declare_schedulable_action(&detail::get_runtime_instance<runtime::LogicalAction>(timer));
}

} // namespace xronos::sdk::detail::runtime_programmable_timer

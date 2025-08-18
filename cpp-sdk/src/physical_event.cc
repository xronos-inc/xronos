// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/physical_event.hh"

#include <any>
#include <string_view>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk::detail::runtime_physical_event {

auto make_instance(std::string_view name, ReactorContext context) -> RuntimeElementPtr {
  return make_runtime_element_pointer<runtime::PhysicalAction>(name, detail::get_reactor_instance(context));
}

void trigger(Element& event, const std::any& value) {
  detail::get_runtime_instance<runtime::PhysicalAction>(event).schedule(value);
}

auto is_present(const Element& event) noexcept -> bool {
  return detail::get_runtime_instance<runtime::PhysicalAction>(event).is_present();
}

auto get(const Element& event) noexcept -> const std::any& {
  return detail::get_runtime_instance<runtime::PhysicalAction>(event).get();
}

void register_as_trigger_of(const Element& event, runtime::Reaction& reaction) noexcept {
  reaction.declare_trigger(&detail::get_runtime_instance<runtime::PhysicalAction>(event));
}

} // namespace xronos::sdk::detail::runtime_physical_event

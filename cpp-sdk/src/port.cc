// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/port.hh"

#include <any>
#include <string_view>

#include "xronos/runtime/port.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/fwd.hh"

namespace xronos::sdk::detail::runtime_port {

auto is_present(const Element& port) noexcept -> bool {
  return detail::get_runtime_instance<runtime::Port>(port).is_present();
}

auto get(const Element& port) noexcept -> const std::any& {
  return detail::get_runtime_instance<runtime::Port>(port).get();
}

void set(Element& port, const std::any& value) noexcept {
  detail::get_runtime_instance<runtime::Port>(port).set(value);
}

void register_as_trigger_of(const Element& port, runtime::Reaction& reaction) noexcept {
  reaction.declare_trigger(&detail::get_runtime_instance<runtime::Port>(port));
}

auto make_input(std::string_view name, ReactorContext context) -> RuntimeElementPtr {
  return make_runtime_element_pointer<runtime::Input>(name, detail::get_reactor_instance(context));
}

auto make_output(std::string_view name, ReactorContext context) -> RuntimeElementPtr {
  return make_runtime_element_pointer<runtime::Output>(name, detail::get_reactor_instance(context));
}

void register_as_effect_of(const Element& port, runtime::Reaction& reaction) noexcept {

  reaction.declare_antidependency(&detail::get_runtime_instance<runtime::Port>(port));
}

} // namespace xronos::sdk::detail::runtime_port

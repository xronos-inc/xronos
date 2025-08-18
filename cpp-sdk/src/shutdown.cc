// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/shutdown.hh"

#include <string_view>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/reaction.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk {

Shutdown::Shutdown(std::string_view name, ReactorContext context)
    : Element{
          detail::make_runtime_element_pointer<runtime::ShutdownTrigger>(name, detail::get_reactor_instance(context)),
          context} {}

[[nodiscard]] auto Shutdown::is_present() const noexcept -> bool {
  return detail::get_runtime_instance<runtime::ShutdownTrigger>(*this).is_present();
}

void Shutdown::register_as_trigger_of(runtime::Reaction& reaction) const noexcept {
  reaction.declare_trigger(&detail::get_runtime_instance<runtime::ShutdownTrigger>(*this));
}

} // namespace xronos::sdk

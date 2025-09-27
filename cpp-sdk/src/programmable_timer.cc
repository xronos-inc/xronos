// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/programmable_timer.hh"

#include <string_view>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"

namespace xronos::sdk::detail {

using CA = ContextAccess;

auto register_programmable_timer(std::string_view name, const ReactorContext& context) -> const core::Element& {
  return CA::get_program_context(context)->model.element_registry.add_new_element(name, core::ProgrammableTimerTag{},
                                                                                  CA::get_parent_uid(context));
}

} // namespace xronos::sdk::detail

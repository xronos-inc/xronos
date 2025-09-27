// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/startup.hh"

#include <string_view>

#include "impl/xronos/sdk/detail/context_access.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk {

using CA = detail::ContextAccess;

Startup::Startup(std::string_view name, ReactorContext context)
    : Element{CA::get_program_context(context)->model.element_registry.add_new_element(name, core::StartupTag{},
                                                                                       CA::get_parent_uid(context)),
              context} {}

} // namespace xronos::sdk

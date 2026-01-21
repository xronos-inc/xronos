// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/startup.hh"

#include <string_view>

#include "impl/xronos/sdk/detail/element.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"
#include "xronos/sdk/element.hh"

namespace xronos::sdk {

Startup::Startup(std::string_view name, ReactorContext context)
    : Element{detail::register_element(name, core::StartupTag{}, context), context} {}

} // namespace xronos::sdk

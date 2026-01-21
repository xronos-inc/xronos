// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/programmable_timer.hh"

#include <string_view>

#include "impl/xronos/sdk/detail/element.hh"
#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"

namespace xronos::sdk::detail {

auto register_programmable_timer(std::string_view name, const ReactorContext& context) -> const core::Element& {
  return register_element(name, core::ProgrammableTimerTag{}, context);
}

} // namespace xronos::sdk::detail

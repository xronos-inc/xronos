// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IMPL_XRONOS_SDK_DETAIL_ELEMENT_HH
#define IMPL_XRONOS_SDK_DETAIL_ELEMENT_HH

#include <string_view>

#include "xronos/core/element.hh"
#include "xronos/sdk/context.hh"

namespace xronos::sdk::detail {

auto register_element(std::string_view name, core::ElementType type, const Context& context) -> const core::Element&;

} // namespace xronos::sdk::detail

#endif // IMPL_XRONOS_SDK_DETAIL_ELEMENT_HH

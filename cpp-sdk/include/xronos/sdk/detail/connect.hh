// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_DETAIL_CONNECT_HH
#define XRONOS_SDK_DETAIL_CONNECT_HH

#include <optional>

#include "xronos/sdk/fwd.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk::detail {

void connect_impl(detail::ProgramContext& context, const Element& from_port, const Element& to_port,
                  const std::optional<Duration>& delay);

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_DETAIL_CONNECT_HH

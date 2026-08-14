// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_SDK_DETAIL_CONNECT_HH
#define XRONOS_SDK_DETAIL_CONNECT_HH

#include <optional>

#include "xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/time.hh"

namespace xronos::sdk::detail {

inline void connect_impl(detail::ProgramContext& program_context, const Element& from_port, const Element& to_port,
                         const std::optional<Duration>& delay) {
  // Throws ValidationError if the downstream port already has a
  // connection, or once a run has been prepared (the implementation seals
  // the model).
  if (delay.has_value()) {
    program_context.backend().add_delayed_connection(from_port.uid(), to_port.uid(), *delay);
  } else {
    program_context.backend().add_connection(from_port.uid(), to_port.uid());
  }
}

} // namespace xronos::sdk::detail

#endif // XRONOS_SDK_DETAIL_CONNECT_HH

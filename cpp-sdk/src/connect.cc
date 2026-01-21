// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/sdk/detail/connect.hh"

#include <cstdint>
#include <optional>

#include "impl/xronos/sdk/detail/program_context.hh"
#include "xronos/sdk/element.hh"
#include "xronos/sdk/environment.hh"
#include "xronos/sdk/time.hh"
#include "xronos/util/assert.hh"

namespace xronos::sdk::detail {

void connect_impl(detail::ProgramContext& program_context, const Element& from_port, const Element& to_port,
                  const std::optional<Duration>& delay) {
  if (program_context.runtime_program_handle != nullptr) {
    throw ValidationError{"Connections may not be created once execution has started."};
  }

  auto& connection_graph = program_context.model.connection_graph;
  if (connection_graph.has_incoming_connection(to_port.uid())) {
    uint64_t upstream_uid = connection_graph.get_upstream_uid(to_port.uid()).value();
    const auto& upstream_port = program_context.model.element_registry.get(upstream_uid);
    throw ValidationError{"Cannot connect port " + from_port.fqn() + " to port " + to_port.fqn() +
                          " because it already has an inbound connection from port " + upstream_port.fqn +
                          ". Each port may have at most one inbound connection."};
  }

  auto success =
      connection_graph.add_connection({.from_uid = from_port.uid(), .to_uid = to_port.uid(), .delay = delay});
  util::assert_(success);
}

} // namespace xronos::sdk::detail

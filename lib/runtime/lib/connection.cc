// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/connection.hh"

#include <vector>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/assert.hh"
#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/port.hh"

namespace xronos::runtime {

void Connection::bind_upstream_port(Port* port) {
  reactor_assert(upstream_port_ == nullptr);
  upstream_port_ = port;
}

void Connection::bind_downstream_ports(const std::vector<Port*>& ports) {
  this->downstream_ports_.insert(ports.begin(), ports.end());
}

void Connection::bind_downstream_port(Port* port) {
  [[maybe_unused]] bool result = this->downstream_ports_.insert(port).second;
  reactor_assert(result);
};

auto BaseDelayedConnection::upstream_set_callback() noexcept -> PortCallback {
  return [this](const Port& port) { this->schedule(port.get()); };
}

void BaseDelayedConnection::setup() noexcept {
  Action::setup();

  for (auto* port : this->downstream_ports()) {
    port->set(this->get());
  }
}

} // namespace xronos::runtime

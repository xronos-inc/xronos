// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/runtime/default/impl/connection.hh"

#include <vector>

#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/fwd.hh"
#include "xronos/runtime/default/impl/port.hh"
#include "xronos/util/assert.hh"

namespace xronos::runtime::default_::impl {

void Connection::bind_upstream_port(Port* port) {
  util::assert_(upstream_port_ == nullptr);
  upstream_port_ = port;
}

void Connection::bind_downstream_ports(const std::vector<Port*>& ports) {
  this->downstream_ports_.insert(ports.begin(), ports.end());
}

void Connection::bind_downstream_port(Port* port) {
  [[maybe_unused]] bool result = this->downstream_ports_.insert(port).second;
  util::assert_(result);
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

} // namespace xronos::runtime::default_::impl

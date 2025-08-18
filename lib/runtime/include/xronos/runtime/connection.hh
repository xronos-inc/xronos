// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2023 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_CONNECTION_HH
#define XRONOS_RUNTIME_CONNECTION_HH

#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "xronos/runtime/action.hh"
#include "xronos/runtime/fwd.hh"
#include "xronos/runtime/port.hh"
#include "xronos/runtime/reactor.hh"
#include "xronos/runtime/time.hh"

namespace xronos::runtime {

class Connection : public Action {
private:
  Port* upstream_port_{nullptr};
  std::set<Port*> downstream_ports_{};

protected:
  using Action::Action;

  [[nodiscard]] auto downstream_ports() -> auto& { return downstream_ports_; }
  [[nodiscard]] auto downstream_ports() const -> const auto& { return downstream_ports_; }
  [[nodiscard]] auto upstream_port() -> auto* { return upstream_port_; }
  [[nodiscard]] auto upstream_port() const -> const auto* { return upstream_port_; }

public:
  virtual auto upstream_set_callback() noexcept -> PortCallback = 0;
  void bind_upstream_port(Port* port);

  void bind_downstream_ports(const std::vector<Port*>& ports);

  void bind_downstream_port(Port* port);

  [[nodiscard]] auto element_type() const -> std::string_view final { return "connection"; };
};

class BaseDelayedConnection : public Connection {
protected:
  using Connection::Connection;

  auto upstream_set_callback() noexcept -> PortCallback final;

public:
  void setup() noexcept final;
};

class DelayedConnection : public BaseDelayedConnection {
public:
  DelayedConnection(const std::string& name, Reactor& container, Duration delay)
      : BaseDelayedConnection(name, container, true, delay) {}
};

class PhysicalConnection : public BaseDelayedConnection {
public:
  PhysicalConnection(const std::string& name, Reactor& container, Duration delay)
      : BaseDelayedConnection(name, container, false, delay) {}
};

} // namespace xronos::runtime

#endif // XRONOS_RUNTIME_CONNECTION_HH

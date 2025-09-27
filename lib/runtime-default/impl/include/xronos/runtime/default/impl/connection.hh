// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-FileCopyrightText: Copyright (c) 2023 TU Dresden
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_IMPL_CONNECTION_HH
#define XRONOS_RUNTIME_DEFAULT_IMPL_CONNECTION_HH

#include <atomic>
#include <cstdint>
#include <limits>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "xronos/core/element.hh"
#include "xronos/runtime/default/impl/action.hh"
#include "xronos/runtime/default/impl/fwd.hh"
#include "xronos/runtime/default/impl/port.hh"
#include "xronos/runtime/default/impl/reactor.hh"
#include "xronos/runtime/default/impl/time.hh"

namespace xronos::runtime::default_::impl {

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
private:
  inline static std::atomic<std::uint64_t> uid_counter_{std::numeric_limits<std::uint64_t>::max()};

protected:
  using Connection::Connection;

  auto upstream_set_callback() noexcept -> PortCallback final;

  // Needed to generate uid for extra elements not managed by
  // core::ElementRegistry. See the comment in DelayedConnection below.
  static auto generate_uid() -> std::uint64_t { return uid_counter_.fetch_add(-1, std::memory_order_relaxed); }

public:
  void setup() noexcept final;
};

class DelayedConnection : public BaseDelayedConnection {
public:
  // We construct a temporary core::Element here. This is necessary to
  // instantiate the connection (as it inherits from ReactorElement). However,
  // we actually do not need to represent the connection as an element and it's
  // name or uid are not used outside of the runtime.
  // This workaround is an artifact of the fact that we (ab)use inheritance to
  // represent the model in the runtime.
  DelayedConnection(const std::string& name, Reactor& container, Duration delay)
      : BaseDelayedConnection(core::Element{.name = name,
                                            .fqn = container.fqn() + '.' + name,
                                            .uid = generate_uid(),
                                            .parent_uid = container.uid(),
                                            .type = core::ProgrammableTimerTag{}},
                              container, true, delay) {}
};

class PhysicalConnection : public BaseDelayedConnection {
public:
  PhysicalConnection(const std::string& name, Reactor& container, Duration delay)
      : BaseDelayedConnection(core::Element{.name = name,
                                            .fqn = container.fqn() + '.' + name,
                                            .uid = generate_uid(),
                                            .parent_uid = container.uid(),
                                            .type = core::PhysicalEventTag{}},
                              container, false, delay) {}
};

} // namespace xronos::runtime::default_::impl

#endif // XRONOS_RUNTIME_DEFAULT_IMPL_CONNECTION_HH

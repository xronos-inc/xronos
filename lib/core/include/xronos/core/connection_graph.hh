// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_CONNECTION_GRAPH_HH
#define XRONOS_CORE_CONNECTION_GRAPH_HH

#include <functional>
#include <optional>
#include <ranges>
#include <unordered_map>

#include "xronos/core/element.hh"
#include "xronos/core/time.hh"

namespace xronos::core {

struct ConnectionProperties {
  ElementID from_uid;
  ElementID to_uid;
  std::optional<Duration> delay;
};

class ConnectionGraph {
public:
  [[nodiscard]] auto has_incoming_connection(ElementID uid) const noexcept -> bool {
    return connections_.contains(uid);
  }
  [[nodiscard]] auto get_upstream_uid(ElementID uid) const noexcept -> std::optional<ElementID>;
  [[nodiscard]] auto get_incoming_connection(ElementID uid) const noexcept
      -> std::optional<std::reference_wrapper<const ConnectionProperties>>;
  auto add_connection(const ConnectionProperties& properties) noexcept -> bool;

  [[nodiscard]] auto connections() const -> auto {
    return connections_ | std::views::transform([](const auto& pair) -> const auto& { return pair.second; });
  }

  // Derives connection properties for end to end connection from its upstream
  // origin to the given downstream port. This accumulates all individual
  // connections found along the way into a single properties object. This
  // assumes that there are no cycles in the connection graph.
  [[nodiscard]] auto get_incoming_end_to_end_connection(ElementID to_uid) const noexcept
      -> std::optional<core::ConnectionProperties>;

private:
  // The to_uid of each connection acts as key. This reflects, that each
  // port may have at most one incoming connection.
  std::unordered_map<ElementID, ConnectionProperties> connections_;
};

} // namespace xronos::core

#endif // XRONOS_CORE_CONNECTION_GRAPH_HH

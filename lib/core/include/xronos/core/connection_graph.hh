// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_CONNECTION_GRAPH_HH
#define XRONOS_CORE_CONNECTION_GRAPH_HH

#include <optional>
#include <ranges>
#include <unordered_map>

#include "xronos/core/element.hh"

namespace xronos::core {

struct ConnectionProperties {
  ElementID from_uid;
  ElementID to_uid;
  std::optional<Duration> delay;
};

class ConnectionGraph {
public:
  auto has_incoming_connection(ElementID uid) const noexcept -> bool { return connections_.contains(uid); }
  auto get_upstream_uid(ElementID uid) const noexcept -> std::optional<ElementID>;
  auto add_connection(const ConnectionProperties& properties) noexcept -> bool;

  [[nodiscard]] auto connections() const -> auto {
    return connections_ | std::views::transform([](const auto& pair) -> const auto& { return pair.second; });
  }

private:
  // The to_uid of each connection acts as key. This reflects, that each
  // port may have at most one incoming connection.
  std::unordered_map<ElementID, ConnectionProperties> connections_;
};

} // namespace xronos::core

#endif // XRONOS_CORE_CONNECTION_GRAPH_HH

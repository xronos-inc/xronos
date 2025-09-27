// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/connection_graph.hh"

#include "xronos/core/element.hh"

#include <optional>

namespace xronos::core {

auto ConnectionGraph::add_connection(const ConnectionProperties& properties) noexcept -> bool {
  auto [_, res] = connections_.try_emplace(properties.to_uid, properties);
  return res;
}

auto ConnectionGraph::get_upstream_uid(ElementID uid) const noexcept -> std::optional<ElementID> {
  auto it = connections_.find(uid);
  if (it == connections_.end()) {
    return std::nullopt;
  }

  return it->second.from_uid;
}

} // namespace xronos::core

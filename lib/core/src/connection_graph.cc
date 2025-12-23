// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/connection_graph.hh"

#include <cstdint>
#include <functional>
#include <optional>

#include "xronos/core/element.hh"
#include "xronos/core/time.hh"

namespace xronos::core {

auto ConnectionGraph::add_connection(const ConnectionProperties& properties) noexcept -> bool {
  auto [_, res] = connections_.try_emplace(properties.to_uid, properties);
  return res;
}

auto ConnectionGraph::get_upstream_uid(ElementID uid) const noexcept -> std::optional<ElementID> {
  if (auto properties = get_incoming_connection(uid); properties.has_value()) {
    return properties.value().get().from_uid;
  }

  return std::nullopt;
}

auto ConnectionGraph::get_incoming_connection(ElementID uid) const noexcept
    -> std::optional<std::reference_wrapper<const ConnectionProperties>> {
  auto it = connections_.find(uid);
  if (it == connections_.end()) {
    return std::nullopt;
  }

  return it->second;
}

auto ConnectionGraph::get_incoming_end_to_end_connection(ElementID to_uid) const noexcept
    -> std::optional<core::ConnectionProperties> {
  std::optional<core::Duration> delay;
  std::uint64_t from_uid = to_uid;
  auto connection = get_incoming_connection(to_uid);
  while (connection.has_value()) {
    const auto& properties = connection->get();
    from_uid = properties.from_uid;
    if (properties.delay.has_value()) {
      if (delay.has_value()) {
        *delay += *properties.delay;
      } else {
        delay = properties.delay;
      }
    }
    connection = get_incoming_connection(from_uid);
  }

  if (from_uid == to_uid) {
    return std::nullopt;
  }

  return core::ConnectionProperties{.from_uid = from_uid, .to_uid = to_uid, .delay = delay};
}

} // namespace xronos::core

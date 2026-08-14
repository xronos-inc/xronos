// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/validator/checks.hh"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "nonstd/expected.hpp"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"

namespace xronos::validator {

namespace {

using OutgoingConnections = std::unordered_map<core::ElementID, std::vector<const core::ConnectionProperties*>>;

// Walks the connections leaving `port_uid` and reports any inconsistencies
// in the boundary crossing markers.
void check_marks(core::ElementID port_uid, bool region_open, const OutgoingConnections& outgoing,
                 const core::ReactorModel& model, std::unordered_set<core::ElementID>& visited,
                 std::vector<std::string>& error_messages) {
  if (!visited.insert(port_uid).second) {
    return;
  }
  auto it = outgoing.find(port_uid);
  if (it == outgoing.end()) {
    return;
  }

  for (const auto* connection : it->second) {
    const auto& from_fqn = model.element_registry.get(connection->from_uid).fqn;
    const auto& to_fqn = model.element_registry.get(connection->to_uid).fqn;
    switch (connection->crossing) {
    case core::BoundaryCrossing::None: {
      check_marks(connection->to_uid, region_open, outgoing, model, visited, error_messages);
      break;
    }
    case core::BoundaryCrossing::Exit: {
      if (region_open) {
        error_messages.push_back(
            fmt::format("The connection from port {} to port {} is marked as a boundary exit, but an upstream "
                        "connection already opened a serialized region.",
                        from_fqn, to_fqn));
      }
      check_marks(connection->to_uid, true, outgoing, model, visited, error_messages);
      break;
    }
    case core::BoundaryCrossing::Entry: {
      if (!region_open) {
        error_messages.push_back(
            fmt::format("The connection from port {} to port {} is marked as a boundary entry, but no upstream "
                        "connection opened a serialized region.",
                        from_fqn, to_fqn));
      }
      check_marks(connection->to_uid, false, outgoing, model, visited, error_messages);
      break;
    }
    case core::BoundaryCrossing::Both: {
      if (region_open) {
        error_messages.push_back(
            fmt::format("The connection from port {} to port {} is marked as a boundary crossing, but an upstream "
                        "connection already opened a serialized region.",
                        from_fqn, to_fqn));
      }
      check_marks(connection->to_uid, false, outgoing, model, visited, error_messages);
      break;
    }
    }
  }
}

} // namespace

// Checks that the boundary crossing marks along every live chain form well
// nested serialized regions. Only chains whose origin port has a writer are
// checked: some reaction must list the origin as an effect for a value to
// ever flow, and a chain nothing writes to delivers nothing, so its marks
// are inert. Structurally valid crossings on such dead chains still go
// through the serializer check, because assembly resolves serializers
// eagerly.
auto check_boundary_crossing_structure(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>> {
  OutgoingConnections outgoing;
  for (const auto& connection : model.connection_graph.connections()) {
    outgoing[connection.from_uid].push_back(&connection);
  }

  std::unordered_set<core::ElementID> written_ports;
  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    for (auto effect : model.reaction_dependency_registry.get_effects(reaction.uid)) {
      written_ports.insert(effect);
    }
  }

  std::vector<std::string> error_messages;
  std::unordered_set<core::ElementID> visited;
  for (const auto& [from_uid, connections] : outgoing) {
    if (model.connection_graph.has_incoming_connection(from_uid)) {
      continue;
    }
    if (!written_ports.contains(from_uid)) {
      continue;
    }
    check_marks(from_uid, false, outgoing, model, visited, error_messages);
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

} // namespace xronos::validator

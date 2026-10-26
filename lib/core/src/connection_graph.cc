// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/connection_graph.hh"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <iterator>
#include <optional>
#include <unordered_map>
#include <vector>

#include "xronos/core/element.hh"
#include "xronos/core/time.hh"

namespace xronos::core {

auto ConnectionGraph::add_connection(const ConnectionProperties& properties) noexcept -> bool {
  auto [_, res] = connections_.try_emplace(properties.to_uid, properties);
  return res;
}

auto ConnectionGraph::get_sending_uid(ElementID uid) const noexcept -> std::optional<ElementID> {
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

namespace {

using OutgoingConnections = std::unordered_map<ElementID, std::vector<const ConnectionProperties*>>;

// Accumulates the delay of one connection onto the delay accumulated so far.
// A connection without a delay leaves the accumulated delay unchanged. This
// matches the delay algebra of get_incoming_end_to_end_connection.
auto accumulate_delay(std::optional<Duration> accumulated, std::optional<Duration> delay) -> std::optional<Duration> {
  if (delay.has_value()) {
    if (accumulated.has_value()) {
      return *accumulated + *delay;
    }
    return delay;
  }
  return accumulated;
}

// Every connection that serializes at the same sending port shares one
// boundary crossing, so the value is serialized once per port.
auto find_or_create_serialization(ConnectionSegment& segment, ElementID serialize_uid) -> ConnectionSerialization& {
  auto serialization = std::ranges::find_if(segment.serializations, [&](const ConnectionSerialization& candidate) {
    return candidate.serialize_uid == serialize_uid;
  });
  if (serialization == segment.serializations.end()) {
    segment.serializations.push_back(ConnectionSerialization{.serialize_uid = serialize_uid, .deserializations = {}});
    serialization = std::prev(segment.serializations.end());
  }
  return *serialization;
}

// Walks the connections leaving `port_uid` and fills in the tree below.
//
// The walk is total. It interprets every combination of marks, including
// ones the validator rejects on live chains, so that computing the trees
// never throws. A `Deserializing` connection with no open crossing adds no
// receiving port and is skipped. A `Serializing` connection inside an open crossing
// acts as a plain connection. A `Complete` connection inside an open
// crossing acts as `Deserializing`.
void collect_receiving_ports(ElementID port_uid, std::optional<Duration> accumulated_delay, ConnectionSegment& segment,
                             ConnectionSerialization* open_serialization, const OutgoingConnections& outgoing) {
  auto it = outgoing.find(port_uid);
  if (it == outgoing.end()) {
    return;
  }

  for (const auto* connection : it->second) {
    auto delay = accumulate_delay(accumulated_delay, connection->delay);
    auto crossing = connection->crossing;
    if (open_serialization != nullptr) {
      if (crossing == BoundaryCrossing::Serializing) {
        crossing = BoundaryCrossing::None;
      } else if (crossing == BoundaryCrossing::Complete) {
        crossing = BoundaryCrossing::Deserializing;
      }
    }

    switch (crossing) {
    case BoundaryCrossing::None: {
      segment.receiving_ports.push_back(ReceivingPort{.port_uid = connection->to_uid, .delay = delay});
      collect_receiving_ports(connection->to_uid, delay, segment, open_serialization, outgoing);
      break;
    }
    case BoundaryCrossing::Serializing: {
      // The receiving port of a serializing connection is a pass-through
      // port and stays a receiving port of the enclosing segment, like any
      // mid-chain port.
      segment.receiving_ports.push_back(ReceivingPort{.port_uid = connection->to_uid, .delay = delay});
      auto& serialization = find_or_create_serialization(segment, connection->from_uid);
      collect_receiving_ports(connection->to_uid, delay, segment, &serialization, outgoing);
      break;
    }
    case BoundaryCrossing::Deserializing: {
      if (open_serialization == nullptr) {
        break;
      }
      auto& deserialization = open_serialization->deserializations.emplace_back(
          ConnectionDeserialization{.deserialize_uid = connection->to_uid, .segment = {}});
      deserialization.segment.receiving_ports.push_back(ReceivingPort{.port_uid = connection->to_uid, .delay = delay});
      collect_receiving_ports(connection->to_uid, delay, deserialization.segment, nullptr, outgoing);
      break;
    }
    case BoundaryCrossing::Complete: {
      auto& serialization = find_or_create_serialization(segment, connection->from_uid);
      auto& deserialization = serialization.deserializations.emplace_back(
          ConnectionDeserialization{.deserialize_uid = connection->to_uid, .segment = {}});
      deserialization.segment.receiving_ports.push_back(ReceivingPort{.port_uid = connection->to_uid, .delay = delay});
      collect_receiving_ports(connection->to_uid, delay, deserialization.segment, nullptr, outgoing);
      break;
    }
    }
  }
}

// Drops boundary crossings that ended up with no deserializing port: a
// crossing that a serializing connection opened and no deserializing
// connection ever closed. No value would pass through such a crossing, and
// the consumers of connection trees rely on every remaining crossing having
// at least one deserializing port.
void prune_serializations_without_deserialization(ConnectionSegment& segment) {
  std::erase_if(segment.serializations,
                [](const ConnectionSerialization& serialization) { return serialization.deserializations.empty(); });
  for (auto& serialization : segment.serializations) {
    for (auto& deserialization : serialization.deserializations) {
      prune_serializations_without_deserialization(deserialization.segment);
    }
  }
}

} // namespace

auto ConnectionGraph::compute_connection_trees() const -> std::vector<ConnectionTree> {
  OutgoingConnections outgoing;
  for (const auto& [to_uid, properties] : connections_) {
    outgoing[properties.from_uid].push_back(&properties);
  }

  std::vector<ConnectionTree> trees;
  for (const auto& [from_uid, connections] : outgoing) {
    if (has_incoming_connection(from_uid)) {
      continue;
    }
    auto& tree = trees.emplace_back(ConnectionTree{.origin_uid = from_uid, .root = {}});
    collect_receiving_ports(from_uid, std::nullopt, tree.root, nullptr, outgoing);
    prune_serializations_without_deserialization(tree.root);
  }

  return trees;
}

} // namespace xronos::core

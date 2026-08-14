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

// All Exit and Both connections leaving the same port share one
// serialization node, so the value is serialized once per port.
auto find_or_create_serialization(DeliverySubtree& subtree, ElementID serialize_uid) -> BoundarySerialization& {
  auto serialization = std::ranges::find_if(subtree.serializations, [&](const BoundarySerialization& candidate) {
    return candidate.serialize_uid == serialize_uid;
  });
  if (serialization == subtree.serializations.end()) {
    subtree.serializations.push_back(BoundarySerialization{.serialize_uid = serialize_uid, .branches = {}});
    serialization = std::prev(subtree.serializations.end());
  }
  return *serialization;
}

// Walks the connections leaving `port_uid` and fills in the tree below.
//
// The walk is total. It interprets every combination of marks, including
// ones the validator rejects on live chains, so that computing the trees
// never throws. An `Entry` with no open region delivers nothing and its
// edge is skipped. An `Exit` inside an open region acts as a plain
// connection. A `Both` inside an open region acts as an `Entry`.
void collect_deliveries(ElementID port_uid, std::optional<Duration> accumulated_delay, DeliverySubtree& subtree,
                        BoundarySerialization* open_serialization, const OutgoingConnections& outgoing) {
  auto it = outgoing.find(port_uid);
  if (it == outgoing.end()) {
    return;
  }

  for (const auto* connection : it->second) {
    auto delay = accumulate_delay(accumulated_delay, connection->delay);
    auto crossing = connection->crossing;
    if (open_serialization != nullptr) {
      if (crossing == BoundaryCrossing::Exit) {
        crossing = BoundaryCrossing::None;
      } else if (crossing == BoundaryCrossing::Both) {
        crossing = BoundaryCrossing::Entry;
      }
    }

    switch (crossing) {
    case BoundaryCrossing::None: {
      subtree.leaves.push_back(DeliveryLeaf{.port_uid = connection->to_uid, .delay = delay});
      collect_deliveries(connection->to_uid, delay, subtree, open_serialization, outgoing);
      break;
    }
    case BoundaryCrossing::Exit: {
      // The port behind the exit is a pass-through boundary port and keeps
      // a plain leaf, like any mid-chain port.
      subtree.leaves.push_back(DeliveryLeaf{.port_uid = connection->to_uid, .delay = delay});
      auto& serialization = find_or_create_serialization(subtree, connection->from_uid);
      collect_deliveries(connection->to_uid, delay, subtree, &serialization, outgoing);
      break;
    }
    case BoundaryCrossing::Entry: {
      if (open_serialization == nullptr) {
        break;
      }
      auto& branch = open_serialization->branches.emplace_back(
          BoundaryDeserialization{.deserialize_uid = connection->to_uid, .subtree = {}});
      branch.subtree.leaves.push_back(DeliveryLeaf{.port_uid = connection->to_uid, .delay = delay});
      collect_deliveries(connection->to_uid, delay, branch.subtree, nullptr, outgoing);
      break;
    }
    case BoundaryCrossing::Both: {
      auto& serialization = find_or_create_serialization(subtree, connection->from_uid);
      auto& branch = serialization.branches.emplace_back(
          BoundaryDeserialization{.deserialize_uid = connection->to_uid, .subtree = {}});
      branch.subtree.leaves.push_back(DeliveryLeaf{.port_uid = connection->to_uid, .delay = delay});
      collect_deliveries(connection->to_uid, delay, branch.subtree, nullptr, outgoing);
      break;
    }
    }
  }
}

// Drops serialization nodes that ended up with no deserializing branch: an
// `Exit` whose region no `Entry` ever closed. Nothing would be delivered
// through such a node, and the runtimes rely on every remaining node having
// at least one branch.
void prune_branchless_serializations(DeliverySubtree& subtree) {
  std::erase_if(subtree.serializations,
                [](const BoundarySerialization& serialization) { return serialization.branches.empty(); });
  for (auto& serialization : subtree.serializations) {
    for (auto& branch : serialization.branches) {
      prune_branchless_serializations(branch.subtree);
    }
  }
}

} // namespace

auto ConnectionGraph::compute_delivery_trees() const -> std::vector<DeliveryTree> {
  OutgoingConnections outgoing;
  for (const auto& [to_uid, properties] : connections_) {
    outgoing[properties.from_uid].push_back(&properties);
  }

  std::vector<DeliveryTree> trees;
  for (const auto& [from_uid, connections] : outgoing) {
    if (has_incoming_connection(from_uid)) {
      continue;
    }
    auto& tree = trees.emplace_back(DeliveryTree{.origin_uid = from_uid, .root = {}});
    collect_deliveries(from_uid, std::nullopt, tree.root, nullptr, outgoing);
    prune_branchless_serializations(tree.root);
  }

  return trees;
}

} // namespace xronos::core

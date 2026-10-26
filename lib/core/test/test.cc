// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// These tests check the connection trees computed by the connection graph.
// The graph stores connections in unordered containers, so the tests never
// rely on the order of trees, receiving ports, or deserializations. They
// look up vertices by uid instead.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/time.hh"

namespace {

using namespace std::chrono_literals;
using xronos::core::BoundaryCrossing;
using xronos::core::ConnectionSegment;
using xronos::core::ConnectionSerialization;
using xronos::core::ConnectionTree;
using xronos::core::ConnectionGraph;
using xronos::core::Duration;
using xronos::core::ElementID;
using xronos::core::ReceivingPort;

auto find_tree(const std::vector<ConnectionTree>& trees, ElementID origin_uid) -> const ConnectionTree& {
  auto it = std::ranges::find_if(trees, [&](const ConnectionTree& tree) { return tree.origin_uid == origin_uid; });
  REQUIRE(it != trees.end());
  return *it;
}

auto find_receiving_port(const ConnectionSegment& segment, ElementID port_uid) -> const ReceivingPort& {
  auto it = std::ranges::find_if(segment.receiving_ports,
                                 [&](const ReceivingPort& port) { return port.port_uid == port_uid; });
  REQUIRE(it != segment.receiving_ports.end());
  return *it;
}

auto find_serialization(const ConnectionSegment& segment, ElementID serialize_uid) -> const ConnectionSerialization& {
  auto it = std::ranges::find_if(segment.serializations, [&](const ConnectionSerialization& serialization) {
    return serialization.serialize_uid == serialize_uid;
  });
  REQUIRE(it != segment.serializations.end());
  return *it;
}

TEST_CASE("A single connection yields one tree with one receiving port", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 1);
  REQUIRE(tree.root.serializations.empty());
  const auto& port = find_receiving_port(tree.root, 2);
  CHECK(!port.delay.has_value());
}

TEST_CASE("Delays accumulate along a chain", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 1s}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 2s}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 2);
  CHECK(find_receiving_port(tree.root, 2).delay == std::optional<Duration>{1s});
  CHECK(find_receiving_port(tree.root, 3).delay == std::optional<Duration>{3s});
}

TEST_CASE("A zero delay stays a zero delay when accumulated", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 0s}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 0s}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  CHECK(find_receiving_port(tree.root, 2).delay == std::optional<Duration>{0s});
  CHECK(find_receiving_port(tree.root, 3).delay == std::optional<Duration>{0s});
}

TEST_CASE("Connections without a delay leave the accumulated delay unchanged", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 1s}));
  REQUIRE(graph.add_connection({.from_uid = 3, .to_uid = 4, .delay = std::nullopt}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  CHECK(!find_receiving_port(tree.root, 2).delay.has_value());
  CHECK(find_receiving_port(tree.root, 3).delay == std::optional<Duration>{1s});
  CHECK(find_receiving_port(tree.root, 4).delay == std::optional<Duration>{1s});
}

TEST_CASE("Each origin port gets its own tree", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(graph.add_connection({.from_uid = 3, .to_uid = 4, .delay = std::nullopt}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 2);
  CHECK(find_tree(trees, 1).root.receiving_ports.size() == 1);
  CHECK(find_tree(trees, 3).root.receiving_ports.size() == 1);
}

TEST_CASE("Cross boundary connections from one port share a boundary crossing", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  CHECK(tree.root.receiving_ports.empty());
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 2);
  for (const auto& deserialization : serialization.deserializations) {
    REQUIRE(deserialization.segment.receiving_ports.size() == 1);
    CHECK(deserialization.segment.receiving_ports.front().port_uid == deserialization.deserialize_uid);
    CHECK(!deserialization.segment.receiving_ports.front().delay.has_value());
  }
}

TEST_CASE("A boundary in the middle of a chain splits the tree", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 1);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  const auto& serialization = find_serialization(tree.root, 2);
  REQUIRE(serialization.deserializations.size() == 1);
  const auto& deserialization = serialization.deserializations.front();
  CHECK(deserialization.deserialize_uid == 3);
  REQUIRE(deserialization.segment.receiving_ports.size() == 1);
  CHECK(deserialization.segment.receiving_ports.front().port_uid == 3);
}

TEST_CASE("Nested boundaries nest in the tree", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  CHECK(tree.root.receiving_ports.empty());
  const auto& outer = find_serialization(tree.root, 1);
  REQUIRE(outer.deserializations.size() == 1);
  const auto& outer_deserialization = outer.deserializations.front();
  CHECK(outer_deserialization.deserialize_uid == 2);
  REQUIRE(outer_deserialization.segment.receiving_ports.size() == 1);
  CHECK(outer_deserialization.segment.receiving_ports.front().port_uid == 2);
  const auto& inner = find_serialization(outer_deserialization.segment, 2);
  REQUIRE(inner.deserializations.size() == 1);
  const auto& inner_deserialization = inner.deserializations.front();
  CHECK(inner_deserialization.deserialize_uid == 3);
  REQUIRE(inner_deserialization.segment.receiving_ports.size() == 1);
  CHECK(inner_deserialization.segment.receiving_ports.front().port_uid == 3);
}

TEST_CASE("A delayed cross boundary connection delays its receiving port", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 5s, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  const auto& deserialization = serialization.deserializations.front();
  REQUIRE(deserialization.segment.receiving_ports.size() == 1);
  CHECK(deserialization.segment.receiving_ports.front().delay == std::optional<Duration>{5s});
}

TEST_CASE("Plain and cross boundary connections mix at one level", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 1);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  CHECK(serialization.deserializations.front().deserialize_uid == 3);
}

TEST_CASE("A serializing connection followed by a deserializing one forms one boundary crossing", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection(
      {.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Deserializing}));

  auto trees = graph.compute_connection_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  // The same serialization structure a single Complete connection from 1 to 3
  // yields, plus a receiving port for the pass-through port behind the
  // serializing port.
  REQUIRE(tree.root.receiving_ports.size() == 1);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  const auto& deserialization = serialization.deserializations.front();
  CHECK(deserialization.deserialize_uid == 3);
  REQUIRE(deserialization.segment.receiving_ports.size() == 1);
  CHECK(deserialization.segment.receiving_ports.front().port_uid == 3);
  CHECK(deserialization.segment.serializations.empty());
}

TEST_CASE("A boundary crossing may span several plain connections", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt}));
  REQUIRE(graph.add_connection(
      {.from_uid = 3, .to_uid = 4, .delay = std::nullopt, .crossing = BoundaryCrossing::Deserializing}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  // Both pass-through ports are receiving ports of the enclosing segment.
  REQUIRE(tree.root.receiving_ports.size() == 2);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  CHECK(find_receiving_port(tree.root, 3).port_uid == 3);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  const auto& deserialization = serialization.deserializations.front();
  CHECK(deserialization.deserialize_uid == 4);
  REQUIRE(deserialization.segment.receiving_ports.size() == 1);
  CHECK(deserialization.segment.receiving_ports.front().port_uid == 4);
}

TEST_CASE("A plain consumer below a serializing port stays outside the boundary crossing", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection(
      {.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Deserializing}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 4, .delay = std::nullopt}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 2);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  CHECK(find_receiving_port(tree.root, 4).port_uid == 4);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  CHECK(serialization.deserializations.front().deserialize_uid == 3);
}

TEST_CASE("A serializing and a both connection from one port share a boundary crossing", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection(
      {.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Deserializing}));
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 4, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 2);
  for (auto deserialize_uid : {ElementID{3}, ElementID{4}}) {
    auto it = std::ranges::find_if(serialization.deserializations, [&](const auto& deserialization) {
      return deserialization.deserialize_uid == deserialize_uid;
    });
    REQUIRE(it != serialization.deserializations.end());
    CHECK(it->segment.receiving_ports.size() == 1);
  }
}

TEST_CASE("A deserializing connection without an open crossing has no receiving port", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Deserializing}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  // Nothing below the deserializing port either: the value would only
  // exist in serialized form past a serializing port, and there is none.
  CHECK(tree.root.receiving_ports.empty());
  CHECK(tree.root.serializations.empty());
}

TEST_CASE("A serializing connection whose crossing is never closed is pruned", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  // The chain behaves like one of plain connections: receiving ports
  // stay, and no boundary crossing without a deserializing port survives.
  REQUIRE(tree.root.receiving_ports.size() == 2);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  CHECK(find_receiving_port(tree.root, 3).port_uid == 3);
  CHECK(tree.root.serializations.empty());
}

TEST_CASE("A serializing connection inside an open crossing acts as a plain connection", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection(
      {.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection(
      {.from_uid = 3, .to_uid = 4, .delay = std::nullopt, .crossing = BoundaryCrossing::Deserializing}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 2);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  CHECK(find_receiving_port(tree.root, 3).port_uid == 3);
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  CHECK(serialization.deserializations.front().deserialize_uid == 4);
}

TEST_CASE("A both connection inside an open crossing acts as deserializing", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection(
      {.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Complete}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.receiving_ports.size() == 1);
  CHECK(find_receiving_port(tree.root, 2).port_uid == 2);
  // One crossing, serialized at 1 and deserialized at 3; no second
  // crossing at 2.
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  CHECK(serialization.deserializations.front().deserialize_uid == 3);
}

TEST_CASE("Delays inside a boundary crossing accumulate onto its receiving ports", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 1s, .crossing = BoundaryCrossing::Serializing}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 2s, .crossing = BoundaryCrossing::Deserializing}));

  auto trees = graph.compute_connection_trees();
  const auto& tree = find_tree(trees, 1);
  CHECK(find_receiving_port(tree.root, 2).delay == std::optional<Duration>{1s});
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.deserializations.size() == 1);
  const auto& deserialization = serialization.deserializations.front();
  REQUIRE(deserialization.segment.receiving_ports.size() == 1);
  CHECK(deserialization.segment.receiving_ports.front().delay == std::optional<Duration>{3s});
}

} // namespace

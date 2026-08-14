// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

// These tests check the delivery trees computed by the connection graph.
// The graph stores connections in unordered containers, so the tests never
// rely on the order of trees, leaves, or branches. They look up nodes by
// uid instead.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/time.hh"

namespace {

using namespace std::chrono_literals;
using xronos::core::BoundaryCrossing;
using xronos::core::BoundarySerialization;
using xronos::core::ConnectionGraph;
using xronos::core::DeliveryLeaf;
using xronos::core::DeliverySubtree;
using xronos::core::DeliveryTree;
using xronos::core::Duration;
using xronos::core::ElementID;

auto find_tree(const std::vector<DeliveryTree>& trees, ElementID origin_uid) -> const DeliveryTree& {
  auto it = std::ranges::find_if(trees, [&](const DeliveryTree& tree) { return tree.origin_uid == origin_uid; });
  REQUIRE(it != trees.end());
  return *it;
}

auto find_leaf(const DeliverySubtree& subtree, ElementID port_uid) -> const DeliveryLeaf& {
  auto it = std::ranges::find_if(subtree.leaves, [&](const DeliveryLeaf& leaf) { return leaf.port_uid == port_uid; });
  REQUIRE(it != subtree.leaves.end());
  return *it;
}

auto find_serialization(const DeliverySubtree& subtree, ElementID serialize_uid) -> const BoundarySerialization& {
  auto it = std::ranges::find_if(subtree.serializations, [&](const BoundarySerialization& serialization) {
    return serialization.serialize_uid == serialize_uid;
  });
  REQUIRE(it != subtree.serializations.end());
  return *it;
}

TEST_CASE("A single connection yields one tree with one leaf", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 1);
  REQUIRE(tree.root.serializations.empty());
  const auto& leaf = find_leaf(tree.root, 2);
  CHECK(!leaf.delay.has_value());
}

TEST_CASE("Delays accumulate along a chain", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 1s}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 2s}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 2);
  CHECK(find_leaf(tree.root, 2).delay == std::optional<Duration>{1s});
  CHECK(find_leaf(tree.root, 3).delay == std::optional<Duration>{3s});
}

TEST_CASE("A zero delay stays a zero delay when accumulated", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 0s}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 0s}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  CHECK(find_leaf(tree.root, 2).delay == std::optional<Duration>{0s});
  CHECK(find_leaf(tree.root, 3).delay == std::optional<Duration>{0s});
}

TEST_CASE("Connections without a delay leave the accumulated delay unchanged", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 1s}));
  REQUIRE(graph.add_connection({.from_uid = 3, .to_uid = 4, .delay = std::nullopt}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  CHECK(!find_leaf(tree.root, 2).delay.has_value());
  CHECK(find_leaf(tree.root, 3).delay == std::optional<Duration>{1s});
  CHECK(find_leaf(tree.root, 4).delay == std::optional<Duration>{1s});
}

TEST_CASE("Each origin port gets its own tree", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(graph.add_connection({.from_uid = 3, .to_uid = 4, .delay = std::nullopt}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 2);
  CHECK(find_tree(trees, 1).root.leaves.size() == 1);
  CHECK(find_tree(trees, 3).root.leaves.size() == 1);
}

TEST_CASE("Cross boundary connections from one port share a serialization node", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  CHECK(tree.root.leaves.empty());
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 2);
  for (const auto& branch : serialization.branches) {
    REQUIRE(branch.subtree.leaves.size() == 1);
    CHECK(branch.subtree.leaves.front().port_uid == branch.deserialize_uid);
    CHECK(!branch.subtree.leaves.front().delay.has_value());
  }
}

TEST_CASE("A boundary in the middle of a chain splits the tree", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 1);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  const auto& serialization = find_serialization(tree.root, 2);
  REQUIRE(serialization.branches.size() == 1);
  const auto& branch = serialization.branches.front();
  CHECK(branch.deserialize_uid == 3);
  REQUIRE(branch.subtree.leaves.size() == 1);
  CHECK(branch.subtree.leaves.front().port_uid == 3);
}

TEST_CASE("Nested boundaries nest in the tree", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  CHECK(tree.root.leaves.empty());
  const auto& outer = find_serialization(tree.root, 1);
  REQUIRE(outer.branches.size() == 1);
  const auto& outer_branch = outer.branches.front();
  CHECK(outer_branch.deserialize_uid == 2);
  REQUIRE(outer_branch.subtree.leaves.size() == 1);
  CHECK(outer_branch.subtree.leaves.front().port_uid == 2);
  const auto& inner = find_serialization(outer_branch.subtree, 2);
  REQUIRE(inner.branches.size() == 1);
  const auto& inner_branch = inner.branches.front();
  CHECK(inner_branch.deserialize_uid == 3);
  REQUIRE(inner_branch.subtree.leaves.size() == 1);
  CHECK(inner_branch.subtree.leaves.front().port_uid == 3);
}

TEST_CASE("A delayed cross boundary connection delays its leaf", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 5s, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  const auto& branch = serialization.branches.front();
  REQUIRE(branch.subtree.leaves.size() == 1);
  CHECK(branch.subtree.leaves.front().delay == std::optional<Duration>{5s});
}

TEST_CASE("Plain and cross boundary connections mix at one level", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt}));
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 1);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  CHECK(serialization.branches.front().deserialize_uid == 3);
}

TEST_CASE("An exit followed by an entry forms one serialization node", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Entry}));

  auto trees = graph.compute_delivery_trees();
  REQUIRE(trees.size() == 1);
  const auto& tree = find_tree(trees, 1);
  // The same serialization structure a single Both connection from 1 to 3
  // yields, plus a plain leaf for the pass-through port behind the exit.
  REQUIRE(tree.root.leaves.size() == 1);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  const auto& branch = serialization.branches.front();
  CHECK(branch.deserialize_uid == 3);
  REQUIRE(branch.subtree.leaves.size() == 1);
  CHECK(branch.subtree.leaves.front().port_uid == 3);
  CHECK(branch.subtree.serializations.empty());
}

TEST_CASE("A serialized region may span several plain connections", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt}));
  REQUIRE(
      graph.add_connection({.from_uid = 3, .to_uid = 4, .delay = std::nullopt, .crossing = BoundaryCrossing::Entry}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  // Both pass-through ports keep plain leaves in the enclosing subtree.
  REQUIRE(tree.root.leaves.size() == 2);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  CHECK(find_leaf(tree.root, 3).port_uid == 3);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  const auto& branch = serialization.branches.front();
  CHECK(branch.deserialize_uid == 4);
  REQUIRE(branch.subtree.leaves.size() == 1);
  CHECK(branch.subtree.leaves.front().port_uid == 4);
}

TEST_CASE("A plain consumer below an exit stays outside the serialization node", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Entry}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 4, .delay = std::nullopt}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 2);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  CHECK(find_leaf(tree.root, 4).port_uid == 4);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  CHECK(serialization.branches.front().deserialize_uid == 3);
}

TEST_CASE("An exit and a both from one port share a serialization node", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Entry}));
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 4, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 2);
  for (auto deserialize_uid : {ElementID{3}, ElementID{4}}) {
    auto it = std::ranges::find_if(serialization.branches,
                                   [&](const auto& branch) { return branch.deserialize_uid == deserialize_uid; });
    REQUIRE(it != serialization.branches.end());
    CHECK(it->subtree.leaves.size() == 1);
  }
}

TEST_CASE("An entry without an open region delivers nothing", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Entry}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  // Nothing below the entry either: the value would only exist in
  // serialized form past an exit, and there is none.
  CHECK(tree.root.leaves.empty());
  CHECK(tree.root.serializations.empty());
}

TEST_CASE("An exit whose region is never closed is pruned", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  // The chain behaves like an unmarked one: plain leaves stay, and no
  // serialization node without a branch survives.
  REQUIRE(tree.root.leaves.size() == 2);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  CHECK(find_leaf(tree.root, 3).port_uid == 3);
  CHECK(tree.root.serializations.empty());
}

TEST_CASE("An exit inside an open region acts as a plain connection", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(
      graph.add_connection({.from_uid = 3, .to_uid = 4, .delay = std::nullopt, .crossing = BoundaryCrossing::Entry}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 2);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  CHECK(find_leaf(tree.root, 3).port_uid == 3);
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  CHECK(serialization.branches.front().deserialize_uid == 4);
}

TEST_CASE("A both inside an open region acts as an entry", "[core]") {
  ConnectionGraph graph;
  REQUIRE(
      graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = std::nullopt, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(
      graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = std::nullopt, .crossing = BoundaryCrossing::Both}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  REQUIRE(tree.root.leaves.size() == 1);
  CHECK(find_leaf(tree.root, 2).port_uid == 2);
  // One region, serialized at 1 and deserialized at 3; no second
  // serialization node at 2.
  REQUIRE(tree.root.serializations.size() == 1);
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  CHECK(serialization.branches.front().deserialize_uid == 3);
}

TEST_CASE("Delays inside a serialized region accumulate onto its leaves", "[core]") {
  ConnectionGraph graph;
  REQUIRE(graph.add_connection({.from_uid = 1, .to_uid = 2, .delay = 1s, .crossing = BoundaryCrossing::Exit}));
  REQUIRE(graph.add_connection({.from_uid = 2, .to_uid = 3, .delay = 2s, .crossing = BoundaryCrossing::Entry}));

  auto trees = graph.compute_delivery_trees();
  const auto& tree = find_tree(trees, 1);
  CHECK(find_leaf(tree.root, 2).delay == std::optional<Duration>{1s});
  const auto& serialization = find_serialization(tree.root, 1);
  REQUIRE(serialization.branches.size() == 1);
  const auto& branch = serialization.branches.front();
  REQUIRE(branch.subtree.leaves.size() == 1);
  CHECK(branch.subtree.leaves.front().delay == std::optional<Duration>{3s});
}

} // namespace

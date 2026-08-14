// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <variant>

#include "xronos/abi/backend.hh"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/dependency_graph/dependency_graph.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/util/logging.hh"
#include "xronos/util/visitor.hh"

namespace xronos::runtime::default_::detail {

namespace {

// The validator rejects a model with missing serializers before a run, so
// this throw is defense in depth for hosts that skip validation.
auto resolve_serializer(const core::ReactorModel& model, core::ElementID uid) -> abi::PortSerializer* {
  const auto& element = model.element_registry.get(uid);
  auto* serializer = core::get_port_properties(element).serializer.get();
  if (serializer == nullptr) {
    throw ValidationError{"Port " + element.fqn + " is on a cross-boundary connection but has no serializer defined."};
  }
  return serializer;
}

auto is_empty(const ResolvedDeliverySubtree& subtree) -> bool {
  return subtree.leaves.empty() && subtree.serializations.empty();
}

// Resolves the serializers of every boundary crossing and prunes the tree
// to the ports the scheduler stores events for: leaves that trigger no
// reaction are dropped, then branches and serialization nodes that lost
// their whole subtree. Pruning is bottom up, so a reaction-less port in the
// middle of a chain never drops a boundary nested below it. Serializers are
// resolved before pruning, so a missing serializer fails init even when the
// crossing delivers to no reaction.
auto resolve_subtree(const core::ReactorModel& model,
                     const std::unordered_map<std::uint64_t, TriggerProperties>& triggers,
                     const core::DeliverySubtree& subtree) -> ResolvedDeliverySubtree {
  ResolvedDeliverySubtree resolved;
  for (const auto& leaf : subtree.leaves) {
    if (triggers.contains(leaf.port_uid)) {
      resolved.leaves.push_back(leaf);
    }
  }
  for (const auto& serialization : subtree.serializations) {
    ResolvedBoundarySerialization resolved_serialization{.serializer =
                                                             resolve_serializer(model, serialization.serialize_uid),
                                                         .serialize_uid = serialization.serialize_uid,
                                                         .branches = {}};
    for (const auto& branch : serialization.branches) {
      auto* deserializer = resolve_serializer(model, branch.deserialize_uid);
      auto resolved_branch_subtree = resolve_subtree(model, triggers, branch.subtree);
      if (!is_empty(resolved_branch_subtree)) {
        resolved_serialization.branches.push_back(
            ResolvedBoundaryDeserialization{.deserializer = deserializer,
                                            .deserialize_uid = branch.deserialize_uid,
                                            .subtree = std::move(resolved_branch_subtree)});
      }
    }
    if (!resolved_serialization.branches.empty()) {
      resolved.serializations.push_back(std::move(resolved_serialization));
    }
  }
  return resolved;
}

} // namespace

void RuntimeModel::init(const core::ReactorModel& model) {
  for (const auto& element : model.element_registry.elements()) {
    std::visit(
        util::Visitor{
            [&](const core::PeriodicTimerTag& type) { periodic_timer_properties[element.uid] = *type.properties; },
            [&]([[maybe_unused]] const core::ShutdownTag& type) { shutdown_trigger_uids.push_back(element.uid); },
            [&]([[maybe_unused]] const core::StartupTag& type) { startup_trigger_uids.push_back(element.uid); },
            [&]([[maybe_unused]] const core::ReactionTag& type) {
              for (auto trigger_uid : model.reaction_dependency_registry.get_triggers(element.uid)) {
                triggers[trigger_uid].triggered_reaction_uids.push_back(element.uid);
              }
            },
            []([[maybe_unused]] const auto& type) {},
        },
        element.type);
  }

  for (const auto& tree : model.connection_graph.compute_delivery_trees()) {
    auto resolved = resolve_subtree(model, triggers, tree.root);
    if (!is_empty(resolved)) {
      delivery_trees.emplace(tree.origin_uid, std::move(resolved));
    }
  }

  dependency_graph::DependencyGraph dependency_graph;
  dependency_graph.init(model);

  auto result = dependency_graph.total_order(model.element_registry);
  if (result.has_value()) {
    ordered_reaction_uids = *result;
  } else {
    util::log::error() << result.error();
    throw ValidationError{"There is a dependency cycle!"};
  }
  if constexpr (util::log::debug_enabled) {
    auto debug = util::log::debug();
    debug << "Total order of all reactions: \n";
    for (auto uid : ordered_reaction_uids) {
      debug << "  - " << model.element_registry.get(uid).fqn << '\n';
    }
  }
}

} // namespace xronos::runtime::default_::detail

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

auto is_empty(const ResolvedConnectionSegment& segment) -> bool {
  return segment.deliveries.empty() && segment.serializations.empty();
}

// Resolves the serializers of every boundary crossing and prunes the tree
// to the ports the scheduler stores events for: receiving ports that trigger
// no reaction are dropped, then deserializing ports and crossings that lost
// their whole segment. Pruning is bottom up, so a reaction-less port in the
// middle of a chain never drops a crossing nested below it. Serializers are
// resolved before pruning, so a missing serializer fails init even when the
// crossing delivers to no reaction.
auto resolve_segment(const core::ReactorModel& model,
                     const std::unordered_map<std::uint64_t, TriggerProperties>& triggers,
                     const core::ConnectionSegment& segment) -> ResolvedConnectionSegment {
  ResolvedConnectionSegment resolved;
  for (const auto& port : segment.receiving_ports) {
    if (triggers.contains(port.port_uid)) {
      resolved.deliveries.push_back(port);
    }
  }
  for (const auto& serialization : segment.serializations) {
    ResolvedConnectionSerialization resolved_serialization{
        .serializer = resolve_serializer(model, serialization.serialize_uid),
        .serialize_uid = serialization.serialize_uid,
        .deserializations = {},
    };
    for (const auto& deserialization : serialization.deserializations) {
      auto* deserializer = resolve_serializer(model, deserialization.deserialize_uid);
      auto resolved_nested_segment = resolve_segment(model, triggers, deserialization.segment);
      if (!is_empty(resolved_nested_segment)) {
        resolved_serialization.deserializations.push_back(ResolvedConnectionDeserialization{
            .deserializer = deserializer,
            .deserialize_uid = deserialization.deserialize_uid,
            .segment = std::move(resolved_nested_segment),
        });
      }
    }
    if (!resolved_serialization.deserializations.empty()) {
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

  for (const auto& tree : model.connection_graph.compute_connection_trees()) {
    auto resolved = resolve_segment(model, triggers, tree.root);
    if (!is_empty(resolved)) {
      connection_trees.emplace(tree.origin_uid, std::move(resolved));
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

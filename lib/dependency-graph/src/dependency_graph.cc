// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/dependency_graph/dependency_graph.hh"
#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <ranges>
#include <sstream>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace xronos::dependency_graph {

void DependencyGraph::init(const core::ReactorModel& model) {
  util::assert_(strong_dependencies_.empty());
  util::assert_(weak_dependencies_.empty());
  util::assert_(!initialized_);
  // ensure that each reaction has an entry in the maps (even if it does not
  // have dependencies).
  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    weak_dependencies_.try_emplace(reaction.uid);
    strong_dependencies_.try_emplace(reaction.uid);
  }

  // add reaction dependencies
  add_intra_rector_dependencies(model.element_registry);
  add_port_dependencies(model);

  if constexpr (util::log::debug_enabled) {
    auto debug = util::log::debug();
    debug << "Strong reaction dependencies:\n";
    for (const auto& [reaction, dependencies] : strong_dependencies_) {
      debug << "  - " << model.element_registry.get(reaction).fqn << " -> [";
      for (auto dep : dependencies) {
        debug << model.element_registry.get(dep.reaction_uid).fqn << ", ";
      }
      debug << "]\n";
    }
    debug << "Weak reaction dependencies:\n";
    for (const auto& [reaction, dependencies] : weak_dependencies_) {
      debug << "  - " << model.element_registry.get(reaction).fqn << " -> [";
      for (auto dep : dependencies) {
        debug << model.element_registry.get(dep.reaction_uid).fqn << " (" << dep.delay << "), ";
      }
      debug << "]\n";
    }
  }

  initialized_ = true;
}

void DependencyGraph::add_intra_rector_dependencies(const core::ElementRegistry& elements) {
  std::unordered_map<std::uint64_t, std::vector<ReactionInfo>> reactions_by_reactor;

  for (const auto& reaction : elements.elements_of_type<core::ReactionTag>()) {
    util::assert_(reaction.parent_uid.has_value());

    // ensure that each reaction has an entry in the map (even if it does not
    // have dependencies).
    reactions_by_reactor[reaction.parent_uid.value()].emplace_back(
        core::get_properties<core::ReactionTag>(reaction).position, reaction.uid);
  }

  for (auto& [_, reactions] : reactions_by_reactor) {
    std::ranges::sort(reactions);
    for (std::uint64_t i{0}; i + 1 < reactions.size(); i++) {
      add_strong_dependency(reactions[i + 1].uid, reactions[i].uid);
    }
    if (reactions.size() > 1) {
      add_weak_dependency(reactions[0].uid, reactions[reactions.size() - 1].uid, core::Duration::zero());
    }
  }
}

void DependencyGraph::add_port_dependencies(const core::ReactorModel& model) {
  ReactionsByElement reactions_by_effect;
  ReactionsByElement reactions_by_trigger;

  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    for (auto trigger : model.reaction_dependency_registry.get_triggers(reaction.uid)) {
      reactions_by_trigger[trigger].push_back(reaction.uid);
    }
    for (auto effect : model.reaction_dependency_registry.get_effects(reaction.uid)) {
      reactions_by_effect[effect].push_back(reaction.uid);
    }
  }

  for (const auto& [trigger_uid, downstream_reactions] : reactions_by_trigger) {
    add_direct_access_dependencies(model, trigger_uid, downstream_reactions, reactions_by_effect);
    add_connection_dependencies(model, trigger_uid, downstream_reactions, reactions_by_effect);
  }
}

// Reactions may access a port directly, without a connection: a parent
// reaction can write a contained reactor's input or trigger on a contained
// output. Any reaction effecting the trigger port then runs before the
// triggered reactions at the same tag. This applies to ports only; events
// scheduled on timers arrive at a later tag, so a shared timer imposes no
// same-tag order.
void DependencyGraph::add_direct_access_dependencies(const core::ReactorModel& model, std::uint64_t trigger_uid,
                                                     const std::vector<std::uint64_t>& downstream_reactions,
                                                     const ReactionsByElement& reactions_by_effect) {
  const auto& trigger_element = model.element_registry.get(trigger_uid);
  if (!std::holds_alternative<core::InputPortTag>(trigger_element.type) &&
      !std::holds_alternative<core::OutputPortTag>(trigger_element.type)) {
    return;
  }
  auto it = reactions_by_effect.find(trigger_uid);
  if (it == reactions_by_effect.end()) {
    return;
  }
  for (auto upstream_reaction : it->second) {
    for (auto downstream_reaction : downstream_reactions) {
      add_strong_dependency(downstream_reaction, upstream_reaction, std::nullopt, trigger_uid);
    }
  }
}

void DependencyGraph::add_connection_dependencies(const core::ReactorModel& model, std::uint64_t trigger_uid,
                                                  const std::vector<std::uint64_t>& downstream_reactions,
                                                  const ReactionsByElement& reactions_by_effect) {
  auto connection = model.connection_graph.get_incoming_end_to_end_connection(trigger_uid);
  if (!connection.has_value()) {
    return;
  }
  auto it = reactions_by_effect.find(connection->from_uid);
  if (it == reactions_by_effect.end()) {
    return;
  }
  for (auto upstream_reaction : it->second) {
    for (auto downstream_reaction : downstream_reactions) {
      if (connection->delay.has_value()) {
        add_weak_dependency(downstream_reaction, upstream_reaction, *connection->delay);
      } else {
        add_strong_dependency(downstream_reaction, upstream_reaction,
                              DependencyConnection{.from_uid = connection->from_uid, .to_uid = trigger_uid});
      }
    }
  }
}

namespace {

// Describes a dependency cycle: the reactions on it, followed by the
// connections its edges ride on and the ports its edges access directly.
auto describe_cycle(const std::vector<std::uint64_t>& cycle, const core::ElementRegistry& elements,
                    const std::unordered_map<std::uint64_t, std::vector<StrongDependency>>& strong_dependencies)
    -> std::string {
  std::stringstream sstream;
  sstream << "There is a dependency cycle involving the following reactions:\n";
  for (auto uid : cycle) {
    sstream << "  - " << elements.get(uid).fqn << '\n';
  }
  std::vector<DependencyConnection> cycle_connections;
  std::vector<std::uint64_t> cycle_ports;
  for (std::size_t i = 0; i < cycle.size(); i++) {
    const auto& dependencies = strong_dependencies.at(cycle[i]);
    auto dependency_uid = cycle[(i + 1) % cycle.size()];
    for (const auto& dep : dependencies) {
      if (dep.reaction_uid != dependency_uid) {
        continue;
      }
      if (dep.via_connection.has_value()) {
        cycle_connections.push_back(*dep.via_connection);
      }
      if (dep.via_port.has_value()) {
        cycle_ports.push_back(*dep.via_port);
      }
    }
  }
  if (!cycle_connections.empty()) {
    sstream << "The cycle runs through the following connections:\n";
    for (const auto& connection : cycle_connections) {
      sstream << "  - " << elements.get(connection.from_uid).fqn << " -> " << elements.get(connection.to_uid).fqn
              << '\n';
    }
  }
  if (!cycle_ports.empty()) {
    sstream << "The cycle runs through direct accesses to the following ports:\n";
    for (auto port_uid : cycle_ports) {
      sstream << "  - " << elements.get(port_uid).fqn << '\n';
    }
  }
  return sstream.str();
}

} // namespace

auto DependencyGraph::total_order(const core::ElementRegistry& elements) const
    -> nonstd::expected<std::vector<std::uint64_t>, std::string> {
  util::assert_(initialized_);

  enum class State : std::uint8_t { Unvisited, InProgress, Done };
  std::unordered_map<std::uint64_t, State> state;
  std::vector<std::uint64_t> order;
  order.reserve(strong_dependencies_.size());
  std::vector<std::uint64_t> path;
  std::vector<std::uint64_t> cycle;

  std::vector<std::uint64_t> nodes;
  nodes.reserve(strong_dependencies_.size());
  for (const auto& [uid, _] : strong_dependencies_) {
    nodes.push_back(uid);
  }
  std::ranges::sort(nodes);

  std::function<void(std::uint64_t)> visit = [&](std::uint64_t uid) -> void {
    auto [it, inserted] = state.try_emplace(uid, State::InProgress);
    if (!inserted) {
      if (it->second == State::InProgress && cycle.empty()) {
        auto cycle_start = std::ranges::find(path, uid);
        cycle.assign(cycle_start, path.end());
      }
      return;
    }
    path.push_back(uid);
    if (auto deps = strong_dependencies_.find(uid); deps != strong_dependencies_.end()) {
      for (const auto& dep : deps->second) {
        visit(dep.reaction_uid);
      }
    }
    path.pop_back();
    it->second = State::Done;
    order.push_back(uid);
  };

  for (auto uid : nodes) {
    visit(uid);
  }

  if (!cycle.empty()) {
    return nonstd::unexpected(describe_cycle(cycle, elements, strong_dependencies_));
  }

  return order;
}

} // namespace xronos::dependency_graph

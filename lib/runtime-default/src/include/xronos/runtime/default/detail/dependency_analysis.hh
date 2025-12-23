// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_RUNTIME_DEFAULT_DETAIL_DEPENDENCY_ANALYSIS_HH
#define XRONOS_RUNTIME_DEFAULT_DETAIL_DEPENDENCY_ANALYSIS_HH

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <optional>
#include <queue>
#include <ranges>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/util/assert.hh"
#include "xronos/util/logging.hh"

namespace xronos::runtime::default_::detail {

class DependencyAnalyzer {
public:
  void init(const core::ReactorModel& model, const std::ranges::range auto& end_to_end_connections)
    requires std::same_as<std::ranges::range_value_t<decltype(end_to_end_connections)>, core::ConnectionProperties>
  {
    // ensure that each reaction has an entry in the maps (even if it does not
    // have dependencies).
    for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
      auto res_dependencies = dependencies_.try_emplace(reaction.uid);
      util::assert_(res_dependencies.second);
      auto res_indegree = indegree_.try_emplace(reaction.uid, 0);
      util::assert_(res_indegree.second);
    }

    // add reaction dependencies
    add_intra_rector_dependencies(model.element_registry);
    add_port_dependencies(model, end_to_end_connections);

    if constexpr (util::log::debug_enabled) {
      auto debug = util::log::debug();
      debug << "Reaction dependencies:\n";
      for (const auto& [reaction, dependencies] : dependencies_) {
        debug << "  - " << model.element_registry.get(reaction).fqn << " (" << indegree_[reaction] << ") -> [";
        for (auto dep : dependencies) {
          debug << model.element_registry.get(dep).fqn << ", ";
        }
        debug << "]\n";
      }
    }
  }

  auto get_reaction_uids_ordered() -> std::vector<std::uint64_t> {

    std::queue<std::uint64_t> queue;

    // Enqueue all nodes with zero indegree
    for (const auto& [node, degree] : indegree_) {
      if (degree == 0) {
        queue.push(node);
      }
    }

    std::vector<std::uint64_t> order;
    order.reserve(indegree_.size());

    while (!queue.empty()) {
      std::uint64_t uid = queue.front();
      queue.pop();
      order.push_back(uid);

      for (std::uint64_t dependency_uid : dependencies_[uid]) {
        auto new_degree = --indegree_[dependency_uid];
        if (new_degree == 0) {
          queue.push(dependency_uid);
        }
      }
    }

    if (order.size() != indegree_.size()) {
      throw ValidationError("The reaction graph contains a dependency cycle!");
    }

    return order;
  }

private:
  std::unordered_map<std::uint64_t, std::unordered_set<std::uint64_t>> dependencies_{};
  std::unordered_map<std::uint64_t, std::uint64_t> indegree_{};

  struct ReactionInfo {
    std::uint32_t pos;
    std::uint64_t uid;

    // Default comparison operator. Since `pos` is defined before `uid`, it will
    // sort reactions of the same reactor by their position.
    auto operator<=>(const ReactionInfo&) const = default;
  };

  void add_dependency(std::uint64_t from_uid, std::uint64_t to_uid) {
    auto result = dependencies_[from_uid].insert(to_uid);
    if (result.second) {
      indegree_[to_uid]++;
    }
  }
  void add_intra_rector_dependencies(const core::ElementRegistry& elements) {
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
        add_dependency(reactions[i].uid, reactions[i + 1].uid);
      }
    }
  }

  void add_port_dependencies(const core::ReactorModel& model, const std::ranges::range auto& end_to_end_connections)
    requires std::same_as<std::ranges::range_value_t<decltype(end_to_end_connections)>, core::ConnectionProperties>
  {
    std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> reactions_by_effect;
    std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> reactions_by_trigger;

    for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
      for (auto trigger : model.reaction_dependency_registry.get_triggers(reaction.uid)) {
        reactions_by_trigger[trigger].push_back(reaction.uid);
      }
      for (auto effect : model.reaction_dependency_registry.get_effects(reaction.uid)) {
        reactions_by_effect[effect].push_back(reaction.uid);
      }
    }

    for (const auto& connection : end_to_end_connections) {
      if (!connection.delay.has_value()) {
        for (auto upstream_reaction : reactions_by_effect[connection.from_uid]) {
          for (auto downstream_reaction : reactions_by_trigger[connection.to_uid]) {
            add_dependency(upstream_reaction, downstream_reaction);
          }
        }
      }
    }
  }
};

} // namespace xronos::runtime::default_::detail

#endif // XRONOS_RUNTIME_DEFAULT_DETAIL_DEPENDENCY_ANALYSIS_HH

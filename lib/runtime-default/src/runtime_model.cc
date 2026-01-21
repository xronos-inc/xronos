// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <optional>
#include <ranges>
#include <variant>

#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/runtime/default/detail/dependency_analysis.hh"
#include "xronos/runtime/default/detail/runtime_model.hh"
#include "xronos/util/logging.hh"
#include "xronos/util/visitor.hh"

namespace xronos::runtime::default_::detail {

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

  for (auto uid : triggers | std::views::keys) {
    auto connection = model.connection_graph.get_incoming_end_to_end_connection(uid);
    if (connection.has_value()) {
      end_to_end_connections[connection->from_uid].push_back(*connection);
    }
  }

  DependencyAnalyzer dependency_analyzer;
  dependency_analyzer.init(model, end_to_end_connections | std::views::values | std::views::join);

  ordered_reaction_uids = dependency_analyzer.get_reaction_uids_ordered();
  if constexpr (util::log::debug_enabled) {
    auto debug = util::log::debug();
    debug << "Total order of all reactions: \n";
    for (auto uid : ordered_reaction_uids) {
      debug << "  - " << model.element_registry.get(uid).fqn << '\n';
    }
  }
}

} // namespace xronos::runtime::default_::detail

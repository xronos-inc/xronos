// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <ranges>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "fmt/format.h"
#include "fmt/ranges.h" // IWYU pragma: keep
#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/validator/checks.hh"

namespace xronos::validator {

namespace {

auto is_port(const core::Element& element) -> bool {
  return std::holds_alternative<core::InputPortTag>(element.type) ||
         std::holds_alternative<core::OutputPortTag>(element.type);
}

// The reactions of one reactor that use one port, split by role.
struct PortUses {
  std::vector<const core::Element*> trigger_of;
  std::vector<const core::Element*> effect_of;
};

} // namespace

// Ports carry messages between reactors, and connections deliver them from
// writers to readers. A reactor that uses one port as both a trigger and an
// effect expects to read back its own writes without a connection. This is
// not permitted by the reactor model.
auto check_port_readback(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  // Reactions using each port, grouped by the reactor the reaction belongs to.
  std::unordered_map<core::ElementID, std::unordered_map<core::ElementID, PortUses>> uses_by_port;
  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    const auto reactor_uid = reaction.parent_uid.value();
    for (const auto uid : model.reaction_dependency_registry.get_triggers(reaction.uid)) {
      if (is_port(model.element_registry.get(uid))) {
        uses_by_port[uid][reactor_uid].trigger_of.push_back(&reaction);
      }
    }
    for (const auto uid : model.reaction_dependency_registry.get_effects(reaction.uid)) {
      if (is_port(model.element_registry.get(uid))) {
        uses_by_port[uid][reactor_uid].effect_of.push_back(&reaction);
      }
    }
  }

  std::vector<std::string> error_messages;
  for (const auto& port : model.element_registry.elements() | std::views::filter(is_port)) {
    const auto port_it = uses_by_port.find(port.uid);
    if (port_it == uses_by_port.end()) {
      continue;
    }
    // Sort the reactors so the error order is stable across runs.
    std::vector<core::ElementID> reactor_uids;
    reactor_uids.reserve(port_it->second.size());
    for (const auto& [reactor_uid, uses] : port_it->second) {
      reactor_uids.push_back(reactor_uid);
    }
    std::ranges::sort(reactor_uids);
    for (const auto reactor_uid : reactor_uids) {
      const auto& uses = port_it->second.at(reactor_uid);
      if (uses.trigger_of.empty() || uses.effect_of.empty()) {
        continue;
      }
      auto fqn_of = [](const core::Element* reaction) -> const std::string& { return reaction->fqn; };
      error_messages.emplace_back(
          fmt::format("Reactions of one reactor may not use a port as both a trigger and an effect. The port {} "
                      "is a trigger of {} and an effect of {}.",
                      port.fqn, fmt::join(std::views::transform(uses.trigger_of, fqn_of), ", "),
                      fmt::join(std::views::transform(uses.effect_of, fqn_of), ", ")));
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

} // namespace xronos::validator

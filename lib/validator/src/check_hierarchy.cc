// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "fmt/format.h"
#include "nonstd/expected.hpp"
#include "xronos/core/connection_graph.hh"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/validator/checks.hh"

namespace xronos::validator {

namespace {

auto is_input(const core::Element& element) -> bool { return std::holds_alternative<core::InputPortTag>(element.type); }

auto is_output(const core::Element& element) -> bool {
  return std::holds_alternative<core::OutputPortTag>(element.type);
}

auto is_port(const core::Element& element) -> bool { return is_input(element) || is_output(element); }

// The reactor containing the element. nullopt names the implicit root that
// contains all top-level elements.
auto reactor_of(const core::Element& element) -> const std::optional<core::ElementID>& { return element.parent_uid; }

// Whether `reactor` is a direct child of `parent`.
auto is_child_of(const core::ElementRegistry& registry, const std::optional<core::ElementID>& reactor,
                 const std::optional<core::ElementID>& parent) -> bool {
  return reactor.has_value() && registry.get(*reactor).parent_uid == parent;
}

// Whether the two reactors are contained in the same reactor. The implicit
// root is contained in nothing, so it only pairs with itself.
auto share_parent(const core::ElementRegistry& registry, const std::optional<core::ElementID>& lhs,
                  const std::optional<core::ElementID>& rhs) -> bool {
  if (!lhs.has_value() || !rhs.has_value()) {
    return lhs == rhs;
  }
  return registry.get(*lhs).parent_uid == registry.get(*rhs).parent_uid;
}

// The element kinds a reaction may use as a trigger when they belong to the
// reaction's own reactor.
auto is_own_trigger_kind(const core::Element& element) -> bool {
  return std::holds_alternative<core::PeriodicTimerTag>(element.type) ||
         std::holds_alternative<core::ProgrammableTimerTag>(element.type) ||
         std::holds_alternative<core::PhysicalEventTag>(element.type) || is_input(element) ||
         std::holds_alternative<core::StartupTag>(element.type) ||
         std::holds_alternative<core::ShutdownTag>(element.type);
}

// The element kinds a reaction may use as an effect when they belong to the
// reaction's own reactor.
auto is_own_effect_kind(const core::Element& element) -> bool {
  return is_output(element) || std::holds_alternative<core::ProgrammableTimerTag>(element.type) ||
         std::holds_alternative<core::ShutdownTag>(element.type);
}

} // namespace

auto check_connection_hierarchy(const core::ElementRegistry& registry, core::ElementID from, core::ElementID to)
    -> std::optional<std::string> {
  const auto& from_element = registry.get(from);
  const auto& to_element = registry.get(to);
  for (const auto* element : {&from_element, &to_element}) {
    if (!is_port(*element)) {
      return fmt::format("Cannot connect {} to {} because {} is a {}; connections link ports.", from_element.fqn,
                         to_element.fqn, element->fqn, core::element_type_as_string(element->type));
    }
  }

  const auto& from_reactor = reactor_of(from_element);
  const auto& to_reactor = reactor_of(to_element);
  bool valid{};
  if (is_output(from_element) && is_input(to_element)) {
    // From one child's output to a child's input, both direct children of
    // the same reactor. This includes a reactor feeding back to itself.
    valid = share_parent(registry, from_reactor, to_reactor);
  } else if (is_input(from_element) && is_input(to_element)) {
    // From a reactor's input down to a direct child's input.
    valid = is_child_of(registry, to_reactor, from_reactor);
  } else if (is_output(from_element) && is_output(to_element)) {
    // From a direct child's output up to the reactor's output.
    valid = is_child_of(registry, from_reactor, to_reactor);
  } else {
    // From a reactor's input straight to its own output.
    valid = from_reactor == to_reactor;
  }
  if (valid) {
    return std::nullopt;
  }
  // The generic rule statement speaks of an enclosing reactor. At the top
  // level that reactor is the implicit root, which the user never sees, so a
  // violation between top-level reactors gets a wording of its own.
  auto is_top_level_reactor = [&](const std::optional<core::ElementID>& reactor) {
    return reactor.has_value() && !registry.get(*reactor).parent_uid.has_value();
  };
  if (is_top_level_reactor(from_reactor) && is_top_level_reactor(to_reactor)) {
    return fmt::format(
        "Cannot connect port {} to port {} because the connection does not respect the reactor hierarchy. A "
        "connection source at the top level must be a top-level reactor's output port, and the target a top-level "
        "reactor's input port.",
        from_element.fqn, to_element.fqn);
  }
  return fmt::format(
      "Cannot connect port {} to port {} because the connection does not respect the reactor hierarchy. A connection "
      "stays within one reactor: it may run from the reactor's input to a direct child's input, from a direct child's "
      "output to the reactor's output, from one child's output to a child's input, or from the reactor's input to the "
      "reactor's output.",
      from_element.fqn, to_element.fqn);
}

auto check_trigger_hierarchy(const core::ElementRegistry& registry, core::ElementID reaction, core::ElementID trigger)
    -> std::optional<std::string> {
  const auto& reaction_element = registry.get(reaction);
  const auto& trigger_element = registry.get(trigger);
  const auto& reactor = reactor_of(reaction_element);
  if (reactor_of(trigger_element) == reactor && is_own_trigger_kind(trigger_element)) {
    return std::nullopt;
  }
  if (is_output(trigger_element) && is_child_of(registry, reactor_of(trigger_element), reactor)) {
    return std::nullopt;
  }
  return fmt::format("Reaction {} may not use {} as a trigger. A reaction may trigger on its own reactor's timers, "
                     "physical events, input ports, startup, and shutdown, or on a direct child reactor's output "
                     "port.",
                     reaction_element.fqn, trigger_element.fqn);
}

auto check_effect_hierarchy(const core::ElementRegistry& registry, core::ElementID reaction, core::ElementID effect)
    -> std::optional<std::string> {
  const auto& reaction_element = registry.get(reaction);
  const auto& effect_element = registry.get(effect);
  const auto& reactor = reactor_of(reaction_element);
  if (reactor_of(effect_element) == reactor && is_own_effect_kind(effect_element)) {
    return std::nullopt;
  }
  if (is_input(effect_element) && is_child_of(registry, reactor_of(effect_element), reactor)) {
    return std::nullopt;
  }
  return fmt::format("Reaction {} may not use {} as an effect. A reaction may affect its own reactor's output ports, "
                     "programmable timers, and shutdown, or a direct child reactor's input port.",
                     reaction_element.fqn, effect_element.fqn);
}

// Connections, triggers, and effects stay within one reactor and reach at
// most one level down, to the ports of a direct child. The backend enforces
// the same rules while the model is built; this check covers models built
// without the backend.
auto check_hierarchy(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;
  const auto& registry = model.element_registry;

  // Sort the connections so the error order is stable across runs.
  std::vector<const core::ConnectionProperties*> connections;
  for (const auto& connection : model.connection_graph.connections()) {
    connections.push_back(&connection);
  }
  std::ranges::sort(connections, {}, [](const auto* connection) { return connection->to_uid; });
  for (const auto* connection : connections) {
    if (auto error = check_connection_hierarchy(registry, connection->from_uid, connection->to_uid)) {
      error_messages.push_back(std::move(*error));
    }
  }

  // Sort the dependencies of each reaction so the error order is stable
  // across runs.
  auto sorted = [](auto&& uids) {
    std::vector<core::ElementID> result{uids.begin(), uids.end()};
    std::ranges::sort(result);
    return result;
  };
  for (const auto& reaction : registry.elements_of_type<core::ReactionTag>()) {
    for (const auto uid : sorted(model.reaction_dependency_registry.get_triggers(reaction.uid))) {
      if (auto error = check_trigger_hierarchy(registry, reaction.uid, uid)) {
        error_messages.push_back(std::move(*error));
      }
    }
    for (const auto uid : sorted(model.reaction_dependency_registry.get_effects(reaction.uid))) {
      if (auto error = check_effect_hierarchy(registry, reaction.uid, uid)) {
        error_messages.push_back(std::move(*error));
      }
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

} // namespace xronos::validator

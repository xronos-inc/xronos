// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/validator/checks.hh"

#include <string>
#include <utility>
#include <vector>

#include "fmt/chrono.h" // IWYU pragma: keep
#include "fmt/format.h"
#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"
#include "xronos/core/time.hh"
#include "xronos/dependency_graph/dependency_graph.hh"

namespace xronos::validator {

namespace detail {

// Utility to accumulate errors
template <typename... Checks> auto run_checks(Checks&&... checks) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;

  auto process_check = [&](auto&& result) {
    if (!result) {
      error_messages.insert(error_messages.end(), result.error().begin(), result.error().end());
    }
  };

  (process_check(std::forward<Checks>(checks)()), ...); // fold-expression, calls all checks in sequence

  if (!error_messages.empty()) {
    return nonstd::unexpected(std::move(error_messages));
  }

  return {};
}

} // namespace detail

auto run_all_checks(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  // The remaining checks assume the shapes the hierarchy rules guarantee.
  // Run on a malformed model, they can throw instead of reporting, so a
  // hierarchy failure gates everything else.
  if (auto result = check_hierarchy(model); !result.has_value()) {
    return result;
  }
  return detail::run_checks(
      [&] { return check_shutdown_reactions(model); }, [&] { return check_periodic_timers(model); },
      [&] { return check_dependency_cycles(model); }, [&] { return check_reaction_handlers(model); },
      [&] { return check_cross_boundary_serializers(model); }, [&] { return check_boundary_crossing_structure(model); },
      [&] { return check_port_readback(model); }, [&] { return check_effects_on_connected_ports(model); });
}

// The runtimes invoke reaction handlers unchecked on the hot path; this
// check is what entitles them to.
auto check_reaction_handlers(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;
  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    if (core::get_properties<core::ReactionTag>(reaction).handler == nullptr) {
      error_messages.push_back(fmt::format("Reaction {} has no handler.", reaction.fqn));
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

auto check_periodic_timers(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;
  for (const auto& timer : model.element_registry.elements_of_type<core::PeriodicTimerTag>()) {
    const auto& properties = core::get_properties<core::PeriodicTimerTag>(timer);
    if (properties.period <= core::Duration::zero()) {
      error_messages.push_back(fmt::format("Timer periods must be greater than zero, but timer {} has a period of {}.",
                                           timer.fqn, properties.period));
    }
    if (properties.offset < core::Duration::zero()) {
      error_messages.push_back(fmt::format("Timer offsets must be positive, but timer {} has a period of {}.",
                                           timer.fqn, properties.offset));
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

// A connected port has exactly one writer: the connection. A reaction effect
// on such a port would race the connection for the port's value within a tag,
// so the model rejects the combination outright.
auto check_effects_on_connected_ports(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;
  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    for (auto effect : model.reaction_dependency_registry.get_effects(reaction.uid)) {
      auto connection = model.connection_graph.get_incoming_connection(effect);
      if (connection.has_value()) {
        error_messages.push_back(
            fmt::format("A port with an incoming connection may not be used as a reaction effect, but reaction {} has "
                        "an effect on port {}, which is connected from {}.",
                        reaction.fqn, model.element_registry.get(effect).fqn,
                        model.element_registry.get(connection->get().from_uid).fqn));
      }
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

auto check_dependency_cycles(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  dependency_graph::DependencyGraph dependency_graph;
  dependency_graph.init(model);
  auto result = dependency_graph.total_order(model.element_registry);
  return result.transform_error([](const std::string& error) { return std::vector<std::string>{error}; })
      .transform([](const auto&) {});
}

} // namespace xronos::validator

// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_VALIDATOR_CHECKS_HH
#define XRONOS_VALIDATOR_CHECKS_HH

#include <optional>
#include <string>
#include <vector>

#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/core/reactor_model.hh"

namespace xronos::validator {

[[nodiscard]] auto run_all_checks(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>>;

[[nodiscard]] auto check_shutdown_reactions(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_periodic_timers(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_dependency_cycles(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_reaction_handlers(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_cross_boundary_serializers(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_boundary_crossing_structure(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_port_readback(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_effects_on_connected_ports(const core::ReactorModel& model)
    -> nonstd::expected<void, std::vector<std::string>>;
[[nodiscard]] auto check_hierarchy(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>>;

// Per-edge hierarchy rules, shared with the backend, which enforces them
// while the model is built. Each returns an error message when the edge
// violates the reactor hierarchy, and nothing when it conforms.
[[nodiscard]] auto check_connection_hierarchy(const core::ElementRegistry& registry, core::ElementID from,
                                              core::ElementID to) -> std::optional<std::string>;
[[nodiscard]] auto check_trigger_hierarchy(const core::ElementRegistry& registry, core::ElementID reaction,
                                           core::ElementID trigger) -> std::optional<std::string>;
[[nodiscard]] auto check_effect_hierarchy(const core::ElementRegistry& registry, core::ElementID reaction,
                                          core::ElementID effect) -> std::optional<std::string>;

} // namespace xronos::validator

#endif // XRONOS_VALIDATOR_CHECKS_HH

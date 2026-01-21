// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/validator/checks.hh"

#include <algorithm>
#include <cstdint>
#include <ranges>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "fmt/format.h"
#include "fmt/ranges.h" // IWYU pragma: keep
#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"
#include "xronos/core/reactor_model.hh"

namespace xronos::validator {

auto check_shutdown_reactions(const core::ReactorModel& model) -> nonstd::expected<void, std::vector<std::string>> {
  std::vector<std::string> error_messages;
  for (const auto& reaction : model.element_registry.elements_of_type<core::ReactionTag>()) {
    auto triggers_view = model.reaction_dependency_registry.get_triggers(reaction.uid);
    if (std::ranges::any_of(triggers_view, [&](std::uint64_t uid) {
          return std::holds_alternative<core::ShutdownTag>(model.element_registry.get(uid).type);
        })) {
      auto effects_view = model.reaction_dependency_registry.get_effects(reaction.uid);
      if (!effects_view.empty()) {
        error_messages.emplace_back(
            fmt::format("Reactions triggered by shutdown may not have any effects. The reaction {} is triggered "
                        "by a shutdown event and has effects on {:n}.",
                        reaction.fqn, std::views::transform(effects_view, [&](std::uint64_t uid) {
                          return model.element_registry.get(uid).fqn;
                        })));
      }
    }
  }

  if (!error_messages.empty()) {
    return nonstd::unexpected{std::move(error_messages)};
  }

  return {};
}

} // namespace xronos::validator

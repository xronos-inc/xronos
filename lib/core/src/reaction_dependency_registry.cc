// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/reaction_dependency_registry.hh"

#include "xronos/core/element.hh"

namespace xronos::core {

auto ReactionDependencyRegistry::get_effects(ElementID reaction_uid) const noexcept -> ElementIDView {
  auto it = effects_.find(reaction_uid);
  if (it == effects_.end()) {
    return ElementIDView{};
  }
  const auto& effects = it->second;
  return ElementIDView{effects.begin(), effects.end()};
}

auto ReactionDependencyRegistry::get_triggers(ElementID reaction_uid) const noexcept -> ElementIDView {
  const auto& it = triggers_.find(reaction_uid);
  if (it == triggers_.end()) {
    return ElementIDView{};
  }
  const auto& triggers = it->second;
  return ElementIDView{triggers.begin(), triggers.end()};
}

} // namespace xronos::core

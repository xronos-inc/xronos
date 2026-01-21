// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_REACTION_DEPENDENCY_REGISTRY_HH
#define XRONOS_CORE_REACTION_DEPENDENCY_REGISTRY_HH

#include <ranges>
#include <unordered_map>
#include <unordered_set>

#include "xronos/core/element.hh"

namespace xronos::core {

class ReactionDependencyRegistry {
public:
  using ElementIDView = std::ranges::subrange<std::unordered_set<ElementID>::const_iterator>;

  void register_reaction_effect(ElementID reaction_uid, ElementID element_uid) noexcept {
    effects_[reaction_uid].insert(element_uid);
  }
  void register_reaction_trigger(ElementID reaction_uid, ElementID element_uid) noexcept {
    triggers_[reaction_uid].insert(element_uid);
  }

  [[nodiscard]] auto get_effects(ElementID reaction_uid) const noexcept -> ElementIDView;
  [[nodiscard]] auto get_triggers(ElementID reaction_uid) const noexcept -> ElementIDView;

private:
  std::unordered_map<ElementID, std::unordered_set<ElementID>> effects_{};
  std::unordered_map<ElementID, std::unordered_set<ElementID>> triggers_{};
};

} // namespace xronos::core

#endif // XRONOS_CORE_REACTION_DEPENDENCY_REGISTRY_HH

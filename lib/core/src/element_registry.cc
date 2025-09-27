// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/element_registry.hh"

#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "xronos/core/element.hh"

namespace xronos::core {

auto ElementRegistry::add_new_element(std::string_view name, ElementType type,
                                      const std::optional<ElementID>& parent_uid) -> const Element& {
  ElementID uid = elements_.size();
  // preferring push_back() here over emplace_back() as it allows for
  // aggregate initialization with designated initializers
  elements_.push_back(Element{.name = std::string{name},
                              .fqn = compute_fqn(name, parent_uid),
                              .uid = uid,
                              .parent_uid = parent_uid,
                              .type = std::move(type)});
  return elements_.back();
}

auto ElementRegistry::compute_fqn(std::string_view name, const std::optional<ElementID>& parent_uid) const
    -> std::string {
  if (!parent_uid.has_value()) {
    return std::string{name};
  }

  std::string_view parent_fqn = this->get(parent_uid.value()).fqn;
  std::string fqn;
  fqn.reserve(parent_fqn.size() + 1 + name.size());
  fqn.append(parent_fqn);
  fqn.push_back('.');
  fqn.append(name);
  return fqn;
};

} // namespace xronos::core

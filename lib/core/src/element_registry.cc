// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/core/element_registry.hh"

#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "fmt/format.h"
#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"

namespace xronos::core {

auto ElementRegistry::add_new_element(std::string_view name, ElementType type,
                                      const std::optional<ElementID>& parent_uid) noexcept
    -> nonstd::expected<std::reference_wrapper<const Element>, std::string> {
  ElementID uid = elements_.size();
  std::string fqn = compute_fqn(name, parent_uid);

  if (fqns_.contains(fqn)) {
    return nonstd::make_unexpected(fmt::format("Cannot create the {} \"{}\" as there already is a {} with this name.",
                                               core::element_type_as_string(type), fqn,
                                               core::element_type_as_string(get_element_by_fqn(fqn).type)));
  }

  // preferring push_back() here over emplace_back() as it allows for
  // aggregate initialization with designated initializers
  elements_.push_back(Element{
      .name = std::string{name}, .fqn = std::move(fqn), .uid = uid, .parent_uid = parent_uid, .type = std::move(type)});
  const auto& element = elements_.back();
  fqns_.insert(element.fqn);
  return elements_.back();
}

auto ElementRegistry::get_element_by_fqn(std::string_view fqn) const -> const Element& {
  for (const Element& element : elements()) {
    if (element.fqn == fqn) {
      return element;
    }
  }
  throw std::out_of_range{fmt::format("\"{}\" does not refer to a known element", fqn)};
}

auto ElementRegistry::compute_fqn(std::string_view name, const std::optional<ElementID>& parent_uid) const
    -> std::string {
  if (!parent_uid.has_value()) {
    return std::string{name};
  }

  std::string_view parent_fqn = this->get(parent_uid.value()).fqn;
  return fmt::format("{}.{}", parent_fqn, name);
};

} // namespace xronos::core

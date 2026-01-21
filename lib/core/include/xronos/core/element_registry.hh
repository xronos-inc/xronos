// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_ELEMENT_REGISTRY_HH
#define XRONOS_CORE_ELEMENT_REGISTRY_HH

#include <deque>
#include <functional>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>
#include <unordered_set>

#include "nonstd/expected.hpp"
#include "xronos/core/element.hh"

namespace xronos::core {

class ElementRegistry {
public:
  [[nodiscard]] auto add_new_element(std::string_view name, ElementType type,
                                     const std::optional<ElementID>& parent_uid) noexcept
      -> nonstd::expected<std::reference_wrapper<const Element>, std::string>;

  [[nodiscard]] auto get(ElementID uid) const -> const Element& { return elements_.at(uid); }

  [[nodiscard]] auto elements() const noexcept { return std::views::all(elements_); }

  template <class T> [[nodiscard]] auto elements_of_type() const noexcept {
    return elements() | std::views::filter([](const auto& elem) { return std::holds_alternative<T>(elem.type); });
  }

private:
  // using a deque because it allows keeping reference to the elements
  std::deque<Element> elements_;
  // We use a std::string_view here to avoid storing full fqns in the set. This
  // is safe as long as we keep the underlying elements in the deque above and
  // never delete elements.
  std::unordered_set<std::string_view> fqns_;

  [[nodiscard]] auto compute_fqn(std::string_view name, const std::optional<ElementID>& parent_uid) const
      -> std::string;

  // Lookup by FQN is not efficient and should only be used in rare cases
  [[nodiscard]] auto get_element_by_fqn(std::string_view fqn) const -> const Element&;
};

} // namespace xronos::core

#endif // XRONOS_CORE_ELEMENT_REGISTRY_HH

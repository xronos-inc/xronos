// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_CORE_ELEMENT_REGISTRY_HH
#define XRONOS_CORE_ELEMENT_REGISTRY_HH

#include <deque>
#include <optional>
#include <ranges>
#include <string>
#include <string_view>

#include "xronos/core/element.hh"

namespace xronos::core {

class ElementRegistry {
public:
  [[nodiscard]] auto add_new_element(std::string_view name, ElementType type,
                                     const std::optional<ElementID>& parent_uid) -> const Element&;

  [[nodiscard]] auto get(ElementID uid) const -> const Element& { return elements_.at(uid); }

  [[nodiscard]] auto elements() const noexcept { return std::views::all(elements_); }

  template <class T> [[nodiscard]] auto elements_of_type() const noexcept {
    return elements() | std::views::filter([](const auto& elem) { return std::holds_alternative<T>(elem.type); });
  }

private:
  // using a deque because it allows keeping reference to the elements
  std::deque<Element> elements_;

  [[nodiscard]] auto compute_fqn(std::string_view name, const std::optional<ElementID>& parent_uid) const
      -> std::string;
};

} // namespace xronos::core

#endif // XRONOS_CORE_ELEMENT_REGISTRY_HH

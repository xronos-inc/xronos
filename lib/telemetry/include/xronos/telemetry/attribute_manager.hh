// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_ATTRIBUTE_MANAGER_HH
#define XRONOS_TELEMETRY_ATTRIBUTE_MANAGER_HH

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <variant>

#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"

namespace xronos::telemetry {

/// A variant type that lists all possible attribute types.
// The order of types matters for the pybind11 bindings. See
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html#c-17-library-containers
using AttributeValue = std::variant<std::string, bool, std::int64_t, double>;

using AttributeMap = std::unordered_map<std::string, AttributeValue>;

class AttributeManager {
public:
  /** Set a single attribute for the given element. */
  void set_attribute(std::uint64_t uid, std::string_view key, const AttributeValue& value);
  /** Set multiple attributes for the given element. */
  void set_attributes(std::uint64_t uid, AttributeMap attributes);

  /** Add a new attribute for the given element.
   *
   * Similar to `set_attribute`, but only adds new attributes that are not previously present.
   * Returns `true` if the attribute was inserted.
   */
  auto add_attribute(std::uint64_t uid, std::string_view key, const AttributeValue& value) -> bool;

  /** Add multiple attributes for the given element.
   *
   * Similar to `set_attributes`, but only adds new attributes that are not previously present.
   * Returns `true` if all attributes were inserted.
   */
  auto add_attributes(std::uint64_t uid, AttributeMap attributes) -> bool;

  /** Retrieve the attributes of the given element.
   *
   * This also includes all attributes of the element's containers. If a
   * attribute key is set for an element and one of its containers, then
   * the element's attribute value takes precedence.
   */
  template <class MapType, class ValueType>
  auto get_attributes_converted(std::uint64_t uid, const core::ElementRegistry& element_registry,
                                const std::function<ValueType(const AttributeValue&)>& convert) const -> MapType;

  auto get_attributes(std::uint64_t uid, const core::ElementRegistry& element_registry) const -> AttributeMap {
    return get_attributes_converted<AttributeMap, AttributeValue>(uid, element_registry,
                                                                  [](const auto& value) { return value; });
  }

private:
  std::unordered_map<std::uint64_t, AttributeMap> attribute_maps_{};
  mutable std::mutex mutex_{};

  auto try_emplace_and_get_attribute_map(std::uint64_t uid) -> AttributeMap&;

  template <class MapType, class ValueType>
  void get_attributes_recursive(std::uint64_t uid, const core::ElementRegistry& element_registry,
                                const std::function<ValueType(const AttributeValue&)>& convert, MapType& result) const;
};

template <class MapType, class ValueType>
auto AttributeManager::get_attributes_converted(std::uint64_t uid, const core::ElementRegistry& element_registry,
                                                const std::function<ValueType(const AttributeValue&)>& convert) const
    -> MapType {
  MapType attributes{};
  get_attributes_recursive(uid, element_registry, convert, attributes);
  return attributes;
}

template <class MapType, class ValueType>
void AttributeManager::get_attributes_recursive(std::uint64_t uid, const core::ElementRegistry& element_registry,
                                                const std::function<ValueType(const AttributeValue&)>& convert,
                                                MapType& result) const {
  {
    std::lock_guard<std::mutex> lock{mutex_};
    auto iterator = attribute_maps_.find(uid);
    if (iterator != attribute_maps_.end()) {
      for (const auto& [key, value] : iterator->second) {
        result.try_emplace(key, convert(value));
      }
    }
  }

  const auto& element_info = element_registry.get(uid);
  if (element_info.parent_uid.has_value()) {
    get_attributes_recursive(element_info.parent_uid.value(), element_registry, convert, result);
  }
}

}; // namespace xronos::telemetry

#endif // XRONOS_TELEMETRY_ATTRIBUTE_MANAGER_HH

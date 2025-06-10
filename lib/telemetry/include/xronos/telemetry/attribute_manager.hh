// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_ATTRIBUTE_MANAGER_HH
#define XRONOS_TELEMETRY_ATTRIBUTE_MANAGER_HH

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>

#include "xronos/runtime/reactor_element.hh"

namespace xronos::telemetry {

/// A variant type that lists all possible attribute types.
// The order of types matters for the pybind11 bindings. See
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html#c-17-library-containers
using AttributeValue = std::variant<std::string, bool, std::int64_t, double>;

using AttributeMap = std::unordered_map<std::string, AttributeValue>;

class AttributeManager {

private:
  std::unordered_map<std::uint64_t, AttributeMap> attribute_maps_{};
  mutable std::mutex mutex_{};

  auto try_emplace_and_get_attribute_map(const runtime::ReactorElement& element) -> AttributeMap&;

public:
  /** Set a single attribute for the given element. */
  void set_attribute(const runtime::ReactorElement& element, std::string_view key, AttributeValue value);
  /** Set multiple attributes for the given element. */
  void set_attributes(const runtime::ReactorElement& element, AttributeMap attributes);

  /** Add a new attribute for the given element.
   *
   * Similar to `set_attribute`, but only adds new attributes that are not previously present.
   * Returns `true` if the attribute was inserted.
   */
  auto add_attribute(const runtime::ReactorElement& element, std::string_view key, const AttributeValue& value) -> bool;

  /** Add multiple attributes for the given element.
   *
   * Similar to `set_attributes`, but only adds new attributes that are not previously present.
   * Returns `true` if all attributes were inserted.
   */
  auto add_attributes(const runtime::ReactorElement& element, AttributeMap attributes) -> bool;

  /** Retrieve the attributes of the given element. */
  auto get_attributes(const runtime::ReactorElement& element) const -> std::optional<AttributeMap>;

  //  * This also includes all attributes of the element's containers. If a
  //  * attribute key is set for an element and one of its containers, then
  //  * the element's attribute value takes precedence.
  //  */
  // auto get_attributes(const runtime::ReactorElement& element) const -> AttributeMap;

  template <class MapType, class ValueType>
  auto
  get_attributes_converted(const runtime::ReactorElement& element,
                           std::function<ValueType(const AttributeValue&)> convert) const -> std::optional<MapType>;
};

template <class MapType, class ValueType>
auto AttributeManager::get_attributes_converted(const runtime::ReactorElement& element,
                                                std::function<ValueType(const AttributeValue&)> convert) const
    -> std::optional<MapType> {
  std::lock_guard<std::mutex> lock{mutex_};
  auto iterator = attribute_maps_.find(element.uid());
  if (iterator != attribute_maps_.end()) {
    MapType attributes{};
    for (const auto& [key, value] : iterator->second) {
      attributes.try_emplace(key, convert(value));
    }
    return attributes;
  }
  return std::nullopt;
}

template <class MapType, class ValueType>
void get_attributes_recursive(const AttributeManager& attribute_manager, const runtime::ReactorElement& element,
                              MapType& attributes, std::function<ValueType(const AttributeValue&)> convert) {
  // If there is an attribute map for the given element, insert all attributes in `attributes`.
  // This only copies attributes for keys that are not already defined in `attributes`. By walking
  // the tree from the leaves to the root, we ensure that leaves take precedence.
  auto element_attributes_opt = attribute_manager.get_attributes_converted<MapType, ValueType>(element, convert);

  if (element_attributes_opt.has_value()) {
    auto& element_attributes = element_attributes_opt.value();
    // Merge the elements attributes with the given attribute map. Any
    // previously defined attributes take precedence over the ones defined by
    // this element.
    attributes.merge(element_attributes);
  }
  if (element.container() != nullptr) {
    get_attributes_recursive<MapType, ValueType>(attribute_manager, *element.container(), attributes, convert);
  }
}

template <class MapType, class ValueType>
auto get_merged_attributes(const AttributeManager& attribute_manager, const runtime::ReactorElement& element,
                           std::function<ValueType(const AttributeValue&)> convert) -> MapType {
  MapType attributes{};
  get_attributes_recursive<MapType, ValueType>(attribute_manager, element, attributes, convert);
  return attributes;
}

}; // namespace xronos::telemetry

#endif // XRONOS_TELEMETRY_ATTRIBUTE_MANAGER_HH

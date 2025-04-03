// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "common.hh"

#include "xronos/runtime/reactor.hh"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/telemetry/attribute_manager.hh"

#include <functional>

namespace xronos::telemetry::otel {

void get_attributes_recursive(const AttributeManager& attribute_manager, const runtime::ReactorElement& element,
                              OtelAttributeMap& attributes) {
  // If there is an attribute map for the given element, insert all attributes in `attributes`.
  // This only copies attributes for keys that are not already defined in `attributes`. By walking
  // the tree from the leaves to the root, we ensure that leaves take precedence.
  auto element_attributes_opt = attribute_manager.get_attributes_converted<OtelAttributeMap, OtelAttributeValue>(
      element, [](const AttributeValue& value) {
        return std::visit([&](const auto& arg) { return OtelAttributeValue{arg}; }, value);
      });

  if (element_attributes_opt.has_value()) {
    auto& element_attributes = element_attributes_opt.value();
    // Merge the elements attributes with the given attribute map. Any
    // previously defined attributes take precedence over the ones defined by
    // this element.
    attributes.merge(element_attributes);
  }
  if (element.container() != nullptr) {
    get_attributes_recursive(attribute_manager, *element.container(), attributes);
  }
}

auto get_merged_attributes(const AttributeManager& attribute_manager,
                           const runtime::ReactorElement& element) -> OtelAttributeMap {
  OtelAttributeMap attributes{};
  get_attributes_recursive(attribute_manager, element, attributes);
  return attributes;
}

auto get_low_cardinality_attributes(const AttributeManager& attribute_manager,
                                    const runtime::ReactorElement& element) -> OtelAttributeMap {
  auto attributes = get_merged_attributes(attribute_manager, element);

  // add additional common xronos low cardinality attributes
  attributes["xronos.fqn"] = element.fqn();
  attributes["xronos.name"] = element.name();
  if (element.container() != nullptr) {
    attributes["xronos.container_fqn"] = element.container()->fqn();
  }
  attributes["xronos.element_type"] = element.element_type();

  return attributes;
}

auto get_attribute_names(const OtelAttributeMap& attributes) -> AttributeNameList {
  AttributeNameList names{attributes.size()};
  std::transform(attributes.begin(), attributes.end(), names.begin(),
                 [](auto& entry) { return std::string_view(entry.first); });

  return names;
}

void set_common_high_cardinality_attributes(const runtime::ReactorElement& element, OtelAttributeMap& attributes) {
  auto logical_time = element.container()->get_tag();
  auto lag = std::chrono::system_clock::now() - logical_time.time_point();
  attributes["xronos.timestamp"] = logical_time.time_point().time_since_epoch().count();
  attributes["xronos.microstep"] = logical_time.micro_step();
  attributes["xronos.lag"] = lag.count();
}

} // namespace xronos::telemetry::otel

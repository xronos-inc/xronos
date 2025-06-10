// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "common.hh"

#include "xronos/runtime/reactor.hh"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/telemetry/attribute_manager.hh"

#include <functional>

namespace xronos::telemetry::otel {

auto get_merged_attributes(const AttributeManager& attribute_manager,
                           const runtime::ReactorElement& element) -> OtelAttributeMap {
  return xronos::telemetry::get_merged_attributes<OtelAttributeMap, OtelAttributeValue>(
      attribute_manager, element, [](const AttributeValue& value) {
        return std::visit([&](const auto& arg) { return OtelAttributeValue{arg}; }, value);
      });
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

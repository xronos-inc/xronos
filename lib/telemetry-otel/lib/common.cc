// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "common.hh"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string_view>
#include <variant>

#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::telemetry::otel {

struct ElementType {
  static constexpr std::string_view INPUT_PORT{"input port"};
  static constexpr std::string_view METRIC{"metric"};
  static constexpr std::string_view OUTPUT_PORT{"output port"};
  static constexpr std::string_view PERIODIC_TIMER{"periodic timer"};
  static constexpr std::string_view PHYSICAL_EVENT{"physical event"};
  static constexpr std::string_view PROGRAMMABLE_TIMER{"programmable timer"};
  static constexpr std::string_view REACTION{"reaction"};
  static constexpr std::string_view REACTOR{"reactor"};
  static constexpr std::string_view SHUTDOWN{"shutdown"};
  static constexpr std::string_view STARTUP{"startup"};

  auto operator()([[maybe_unused]] const core::InputPortTag& tag) -> std::string_view { return INPUT_PORT; }
  auto operator()([[maybe_unused]] const core::MetricTag& tag) -> std::string_view { return METRIC; }
  auto operator()([[maybe_unused]] const core::OutputPortTag& tag) -> std::string_view { return OUTPUT_PORT; }
  auto operator()([[maybe_unused]] const core::PeriodicTimerTag& tag) -> std::string_view { return PERIODIC_TIMER; }
  auto operator()([[maybe_unused]] const core::PhysicalEventTag& tag) -> std::string_view { return PHYSICAL_EVENT; }
  auto operator()([[maybe_unused]] const core::ProgrammableTimerTag& tag) -> std::string_view {
    return PROGRAMMABLE_TIMER;
  }
  auto operator()([[maybe_unused]] const core::ReactionTag& tag) -> std::string_view { return REACTION; }
  auto operator()([[maybe_unused]] const core::ReactorTag& tag) -> std::string_view { return REACTOR; }
  auto operator()([[maybe_unused]] const core::ShutdownTag& tag) -> std::string_view { return SHUTDOWN; }
  auto operator()([[maybe_unused]] const core::StartupTag& tag) -> std::string_view { return STARTUP; }
};

auto get_merged_attributes(const AttributeManager& attribute_manager, const core::ElementRegistry& element_registry,
                           std::uint64_t uid) -> OtelAttributeMap {
  return attribute_manager.get_attributes_converted<OtelAttributeMap, OtelAttributeValue>(
      uid, element_registry, [](const AttributeValue& value) {
        return std::visit([&](const auto& arg) { return OtelAttributeValue{arg}; }, value);
      });
}

auto get_low_cardinality_attributes(const AttributeManager& attribute_manager,
                                    const core::ElementRegistry& element_registry, const core::Element& element)
    -> OtelAttributeMap {
  auto attributes = get_merged_attributes(attribute_manager, element_registry, element.uid);

  // add additional common xronos low cardinality attributes
  attributes["xronos.fqn"] = element.fqn;
  attributes["xronos.name"] = element.name;
  std::string_view fqn_view{element.fqn};
  if (auto pos = fqn_view.rfind('.'); pos != std::string_view::npos) {
    attributes["xronos.container_fqn"] = fqn_view.substr(0, pos);
  }
  attributes["xronos.element_type"] = std::visit(ElementType{}, element.type);

  return attributes;
}

auto get_attribute_names(const OtelAttributeMap& attributes) -> AttributeNameList {
  AttributeNameList names{attributes.size()};
  std::ranges::transform(attributes, names.begin(), [](auto& entry) { return std::string_view(entry.first); });

  return names;
}

void set_common_high_cardinality_attributes(const runtime::TimeAccess& time_access, OtelAttributeMap& attributes) {
  auto logical_time = time_access.get_timestamp();
  auto lag = std::chrono::system_clock::now() - logical_time;
  attributes["xronos.timestamp"] = logical_time.time_since_epoch().count();
  attributes["xronos.microstep"] = time_access.get_microstep();
  attributes["xronos.lag"] = lag.count();
}

} // namespace xronos::telemetry::otel

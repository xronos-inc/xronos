// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef COMMON_HH
#define COMMON_HH

#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "opentelemetry/common/attribute_value.h"
#include "opentelemetry/trace/provider.h"
#include "xronos/runtime/reactor_element.hh"
#include "xronos/telemetry/attribute_manager.hh"

namespace xronos::telemetry::otel {

using AttributeNameList = std::vector<std::string_view>;
using OtelAttributeValue = ::opentelemetry::common::AttributeValue;
using OtelAttributeMap = std::unordered_map<std::string, OtelAttributeValue>;

auto get_merged_attributes(const AttributeManager& attribute_manager, const runtime::ReactorElement& element)
    -> OtelAttributeMap;
auto get_low_cardinality_attributes(const AttributeManager& attribute_manager, const runtime::ReactorElement& element)
    -> OtelAttributeMap;
auto get_attribute_names(const OtelAttributeMap& attributes) -> AttributeNameList;

void set_common_high_cardinality_attributes(const runtime::ReactorElement& element, OtelAttributeMap& attributes);

inline auto get_tracer() -> auto { return opentelemetry::trace::Provider::GetTracerProvider()->GetTracer("xronos"); }

} // namespace xronos::telemetry::otel

#endif // COMMON_HH

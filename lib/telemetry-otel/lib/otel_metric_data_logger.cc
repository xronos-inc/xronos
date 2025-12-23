// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#include "xronos/telemetry/otel/otel_metric_data_logger.hh"

#include <chrono>
#include <variant>

#include "common.hh"
#include "opentelemetry/trace/tracer.h"
#include "xronos/core/element.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/telemetry/metric.hh"

using namespace xronos::telemetry;
using namespace xronos::telemetry::otel;

void OtelMetricDataLogger::record(const core::Element& metric, const runtime::TimeAccess& time_access,
                                  MetricValue value) {
  auto current_span = opentelemetry::trace::Tracer::GetCurrentSpan();
  if (!current_span->GetContext().IsValid()) {
    // Either tracing is disabled or there is no current span. The latter case
    // indicates user error, but it is nontrivial to distinguish between the
    // two, and there are other mechanisms to make this user error unlikely, so
    // we do not throw an exception here.
    return;
  }

  const auto& metric_properties = *std::get<core::MetricTag>(metric.type).properties;

  auto attributes = get_low_cardinality_attributes(attribute_manager_, element_registry_, metric);

  // add additional low cardinality attributes
  attributes["xronos.unit"] = metric_properties.unit;

  // Compute list of low cardinality attributes. This assumes that all
  // attributes set so far are low cardinality. We need to explicitly own this
  // list here until we set the attributes, as the attribute map only stores a
  // (non-owning) span.
  auto low_cardinality_attributes = get_attribute_names(attributes);
  attributes["xronos.schema.low_cardinality_attributes"] = low_cardinality_attributes;

  // add the metric value
  attributes["xronos.value"] = std::visit([](const auto& arg) { return OtelAttributeValue{arg}; }, value);

  // add additional high cardinality attributes
  set_common_high_cardinality_attributes(time_access, attributes);
  attributes["xronos.description"] = metric_properties.description;

  // log the event
  auto timestamp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(time_access.get_timestamp());
  current_span->AddEvent(metric.fqn, timestamp, attributes);
}

// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_OTEL_OTEL_METRIC_DATA_LOGGER_HH
#define XRONOS_TELEMETRY_OTEL_OTEL_METRIC_DATA_LOGGER_HH

#include <functional>

#include "xronos/core/element.hh"
#include "xronos/core/element_registry.hh"
#include "xronos/runtime/interfaces.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"

namespace xronos::telemetry::otel {

class OtelMetricDataLogger : public xronos::telemetry::MetricDataLogger {
public:
  OtelMetricDataLogger(const xronos::telemetry::AttributeManager& attribute_manager,
                       const core::ElementRegistry& element_registry)
      : attribute_manager_(attribute_manager)
      , element_registry_(element_registry) {}

  void record(const core::Element& metric, const runtime::TimeAccess& time_access, MetricValue value) final;

private:
  std::reference_wrapper<const AttributeManager> attribute_manager_;
  std::reference_wrapper<const core::ElementRegistry> element_registry_;
};

} // namespace xronos::telemetry::otel

#endif

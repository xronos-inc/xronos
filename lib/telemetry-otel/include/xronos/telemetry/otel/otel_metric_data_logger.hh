// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_OTEL_OTEL_METRIC_DATA_LOGGER_HH
#define XRONOS_TELEMETRY_OTEL_OTEL_METRIC_DATA_LOGGER_HH

#include <functional>

#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"

namespace xronos::telemetry::otel {

class OtelMetricDataLogger : public xronos::telemetry::MetricDataLogger {
private:
  std::reference_wrapper<xronos::telemetry::AttributeManager> attribute_manager_;

public:
  OtelMetricDataLogger(xronos::telemetry::AttributeManager& attribute_manager)
      : attribute_manager_(attribute_manager) {}

  void record(const Metric& metric, MetricValue value) final;
};

} // namespace xronos::telemetry::otel

#endif

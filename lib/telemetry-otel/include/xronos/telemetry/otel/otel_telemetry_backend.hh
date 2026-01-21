// SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_OTEL_OTEL_TELEMETRY_BACKEND_HH
#define XRONOS_TELEMETRY_OTEL_OTEL_TELEMETRY_BACKEND_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/core/element_registry.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/otel/otel_metric_data_logger.hh"
#include "xronos/telemetry/otel/otel_reaction_span_logger.hh"
#include "xronos/telemetry/reaction.hh"
#include "xronos/telemetry/telemetry.hh"

namespace xronos::telemetry::otel {

class OtelTelemetryBackend final : public TelemetryBackend {
public:
  OtelTelemetryBackend(const AttributeManager& attribute_manager, const core::ElementRegistry& element_registry,
                       std::string_view application_name, std::string_view endpoint, std::string_view hostname,
                       std::int64_t pid)
      : reaction_span_logger_(attribute_manager, element_registry)
      , metric_data_logger_(attribute_manager, element_registry)
      , application_name_(application_name)
      , endpoint_(endpoint)
      , hostname_(hostname)
      , pid_(pid) {}

  ~OtelTelemetryBackend() final;

  OtelTelemetryBackend(const OtelTelemetryBackend&) = delete;
  OtelTelemetryBackend(OtelTelemetryBackend&&) = delete;
  auto operator=(const OtelTelemetryBackend&) -> OtelTelemetryBackend& = delete;
  auto operator=(OtelTelemetryBackend&&) -> OtelTelemetryBackend& = delete;

  void initialize() final;
  auto reaction_span_logger() -> ReactionSpanLogger& final { return reaction_span_logger_; }
  auto metric_data_logger() -> MetricDataLogger& final { return metric_data_logger_; }

private:
  OtelReactionSpanLogger reaction_span_logger_;
  OtelMetricDataLogger metric_data_logger_;
  std::string application_name_;
  std::string endpoint_;
  std::string hostname_;
  std::int64_t pid_;
};

} // namespace xronos::telemetry::otel

#endif // XRONOS_TELEMETRY_OTEL_OTEL_TELEMETRY_BACKEND_HH

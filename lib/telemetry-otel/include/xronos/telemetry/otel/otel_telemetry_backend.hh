// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_OTEL_OTEL_TELEMETRY_BACKEND_HH
#define XRONOS_TELEMETRY_OTEL_OTEL_TELEMETRY_BACKEND_HH

#include <cstdint>
#include <string>
#include <string_view>

#include "xronos/runtime/data_logger.hh"
#include "xronos/telemetry/attribute_manager.hh"
#include "xronos/telemetry/metric.hh"
#include "xronos/telemetry/otel/otel_metric_data_logger.hh"
#include "xronos/telemetry/otel/otel_runtime_data_logger.hh"
#include "xronos/telemetry/telemetry.hh"

namespace xronos::telemetry::otel {

class OtelTelemetryBackend final : public TelemetryBackend {
private:
  OtelRuntimeDataLogger runtime_data_logger_;
  OtelMetricDataLogger metric_data_logger_;
  std::string application_name_;
  std::string endpoint_;
  std::string hostname_;
  std::int64_t pid_;

public:
  OtelTelemetryBackend(AttributeManager& attribute_manager, std::string_view application_name,
                       std::string_view endpoint, std::string_view hostname, std::int64_t pid)
      : runtime_data_logger_(attribute_manager)
      , metric_data_logger_(attribute_manager)
      , application_name_(application_name)
      , endpoint_(endpoint)
      , hostname_(hostname)
      , pid_(pid) {}

  ~OtelTelemetryBackend() final { shutdown(); }

  OtelTelemetryBackend(const OtelTelemetryBackend&) = delete;
  OtelTelemetryBackend(OtelTelemetryBackend&&) = delete;
  auto operator=(const OtelTelemetryBackend&) -> OtelTelemetryBackend& = delete;
  auto operator=(OtelTelemetryBackend&&) -> OtelTelemetryBackend& = delete;

  void initialize() final;
  void shutdown() final;
  auto runtime_data_logger() -> runtime::RuntimeDataLogger& final { return runtime_data_logger_; }
  auto metric_data_logger() -> MetricDataLogger& final { return metric_data_logger_; }
};

} // namespace xronos::telemetry::otel

#endif // XRONOS_TELEMETRY_OTEL_OTEL_TELEMETRY_BACKEND_HH

// SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XRONOS_TELEMETRY_TELEMETRY_HH
#define XRONOS_TELEMETRY_TELEMETRY_HH

#include "metric.hh"
#include "xronos/runtime/data_logger.hh"

namespace xronos::telemetry {

class TelemetryBackend {
public:
  virtual ~TelemetryBackend() = default;
  virtual void initialize() = 0;
  virtual void shutdown() = 0;
  virtual auto runtime_data_logger() -> runtime::RuntimeDataLogger& = 0;
  virtual auto metric_data_logger() -> MetricDataLogger& = 0;
};

class NoopTelemetryBackend : public TelemetryBackend {
  runtime::NoopRuntimeDataLogger runtime_data_logger_{};
  NoopMetricDataLogger metric_data_logger_{};

public:
  void initialize() final {};
  void shutdown() final {};
  auto runtime_data_logger() -> runtime::RuntimeDataLogger& final { return runtime_data_logger_; }
  auto metric_data_logger() -> MetricDataLogger& final { return metric_data_logger_; }
};

} // namespace xronos::telemetry

#endif
